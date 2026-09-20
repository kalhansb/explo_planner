# Evidence comments in `explo_planner_node.cpp`

The planner's source carries a large body of comments that are not
descriptions of what the code does but **records of why it does it** —
measurements, counts and outcomes taken from banked simulation campaigns.
This document collects them in one place.

| | |
|---|---|
| Source | `ws/src/explo_planner/explo_planner/src/explo_planner_node.cpp` |
| Revision | `133214c` plus uncommitted working-tree changes |
| Extracted | 2026-09-20 |
| Blocks | 141 |
| Evidence comment lines | 1619 (of 3494 lines in those blocks) |
| Campaigns cited | 14 distinct |

## Why this document exists

An evidence comment is an **unversioned claim about versioned data**. The
comment lives in the source tree; the campaign it cites lives in
`~/hmr_campaign`, outside the repository, and nothing recomputes the claim
when that data is superseded. Three failure modes follow, and the package
already shows all three:

- **The claim cannot be re-checked.** `the nav global planner never plans
  (66/66 robot-logs)` at line 3175 rests on the *absence* of a log line,
  which cannot separate "never planned" from "never logged". Establishing
  it either way needs a same-binary control that was never run.
- **The claim dates itself and is never revisited.** The block at line 2246
  is labelled `INERT SINCE GENERATION 25` — it documents its own expiry and
  has outlived five further generations in the source.
- **The claim is falsifiable and false.** `global_allocator.hpp` asserts
  that every arm of every campaign runs `global_alloc_enable=true`, and
  concludes there is nothing to fix while no such campaign exists. Across
  1751 banked manifests, 60 of 2532 robot-runs ran it false — the whole
  `mh1` off arm, 30 cells. That is the only claim in the package that has
  been checked end to end, and it did not survive.

Collecting them here separates the two jobs the comments were doing. The
source keeps the explanation of the mechanism; this document keeps the
evidence, with the line it came from, so a claim can be re-checked against
the campaign corpus without reading 19,000 lines of C++.

## Campaigns cited

Every campaign token that appears in an evidence comment, with the number
of blocks citing it. A token appearing here is a claim that the campaign's
banked data supports the design decision at that line.

| Campaign | Blocks citing it |
|---|---:|
| `ts4` | 28 |
| `ts1b` | 7 |
| `g8r1` | 5 |
| `g6pilot` | 3 |
| `p7modes` | 3 |
| `cr5` | 3 |
| `cr3` | 2 |
| `ts3` | 1 |
| `p3b` | 1 |
| `p4mild` | 1 |
| `p8trigger` | 1 |
| `tr1` | 1 |
| `mh1` | 1 |
| `gt1` | 1 |

## Distribution

| Region of the file | Blocks |
|---|---:|
| Class declaration — member and method documentation | 44 |
| Constructor — parameter declaration and validation | 22 |
| Runtime logic — planner, manoeuvres and telemetry | 75 |

---

# Class declaration — member and method documentation

### `explo_planner_node.cpp:298-301`

**Documents:** `bool maybeLatchCoverageDone(double unk, const char* source);`

**Cites:** measured

```
Latched, state-blind completion test (done_criterion == "latch"). Takes an
already-measured ROI unknown fraction — every caller has just computed one
and a second ROI walk is the most expensive thing in the tick. Returns
true on the call that latched, false on every other call.
```

### `explo_planner_node.cpp:551-582`

**Documents:** `void maintainRendezvousProposal(bool team_mutual);`

**Cites:** measured

```
ECHO does not, and gating it was a deadlock: a follower could only adopt
the proposer's triple while the WHOLE team was mutually in contact, but
the commit it feeds needs every follower to have already adopted. At N=2
the two conditions coincide and it worked; at N>=3 the fleet is almost
never mutually whole for the two-plus TeamWorld periods a
propose->echo->commit round trip costs, so nothing was ever agreed and
every arming refused (measured: 0 agreements and 18/18 refusals over a
600 s 3-robot cell).

Echoing early is therefore not a private input and does not weaken the
agreement: a follower adopts the proposer's three integers VERBATIM. All
it buys is that the fleet is already converged when a brief whole-team
window opens, so that window only has to be long enough to commit in —
not long enough to negotiate in. At N=4 that window was measured at about
two seconds.
```

### `explo_planner_node.cpp:594-612`

**Documents:** `bool armAppointment(const char* reason);`

**Cites:** campaign, count, `ts4`

```
Turn the committed triple into a standing appointment. THE PLACE AND THE
TIME ARE BOTH THE TRIPLE'S: `cell` is adopted verbatim off the wire and
`t_meet_ms` is an occurrence of the agreed recurrence — the first one this
robot can still ARRIVE at within rendezvous_max_lateness_sec
(nextAgreedOccurrence, floored at t_now plus this robot's shortfall, which
is zero unless it genuinely cannot make the nearest rung). Every robot
that can reach the agreed instant attends the SAME instant; a robot that
cannot, and a robot arming after the instant has genuinely passed, slip by
whole intervals. (Until generation 25 the floor was
now + rendezvous_depart_delay_sec — a lead time added UNCONDITIONALLY,
which forked the ts4 N=3 cell across two occurrences with every robot able
to make the first. That is the distinction the shortfall's clamp at zero
keeps; see the block at the assignment. It described a private countdown
here until generation 23 — see nextAgreedOccurrence in planner_util.hpp
for why that was not a rendezvous.) `interval_ms` is the recurrence period
that occurrence is taken from. Emits one
`rendezvous_agreed` per call, refusals included. Returns true when an
appointment now stands. Does NOT solve — see the P5 state block for why
deriving twice was abandoned.
```

### `explo_planner_node.cpp:723-732`

**Documents:** `void homeWatchdogFire(const char* kind, float metric, float dist_home,`

**Cites:** banked, campaign, count, `g6pilot`

```
`test_delta` is the LEFT-HAND SIDE of the inequality the firing detector
evaluated, selected by the caller to match `kind`: window movement for
"frozen", closing distance for "approach". It is not interchangeable with
`metric`, which is an instantaneous remaining distance sampled at the fire
instant — on the 3 banked g6pilot fires that log both, the two differ by
4.1x, 123x and 162x, and once in sign (the robot was receding). The other
4 banked fires are retrace-mode and log no delta at all, which is why this
parameter exists; see logHomeWatchdog in experiment_log.hpp for the full
accounting, including the two ways this range has been miscounted. The
matching threshold is re-derived inside from `kind`.
```

### `explo_planner_node.cpp:1054-1089`

**Documents:** `CellWorld               rendezvous_world_;`

**Cites:** campaign, count, stat, `ts3`

```
  ts3 n2+n3, every separated pair that armed at both ends
    same (cell, interval):        21 of 64 pairs (33%)
    median t_meet disagreement:   91 s at N=2, 125 s at N=3 (max 496 s)
    N=3 alone:                    0 of 7 pairs agreed
    both ends fell to the floor:  2 of 6 agreed — so even the midpoint
                                  floor is not symmetric
    identical shared_hash:        still 9 of 24 disagreed, and the
                                  candidate COUNT differed in 5 of 19
```

### `explo_planner_node.cpp:1095-1101`

**Documents:** `struct RendezvousProposal {`

**Cites:** measured

```
A (cell, interval) pair, compared as exact integers. `interval_ms` is
measured FROM THE SEPARATION, not from any robot's clock origin, which is
what makes it echoable: the same two numbers mean the same appointment on
every robot, and each end converts to its own t_meet by adding the instant
its own view of the team went incomplete.
```

### `explo_planner_node.cpp:1105-1139`

**Documents:** `long long t_meet_ms   = -1;`

**Cites:** campaign, measured, `ts4`

```
WHY AN INSTANT AND NOT JUST THE INTERVAL (2026-09-17). The interval was
origin-free by design, and each robot supplied its own origin: the
instant its view of the team stopped being mutually whole. That is one
physical event at N=2 and the ts4 smoke measured the two ends 0.84 s
apart — but it is NOT one event at N>=3, because "every one of MY links
is up" is a per-robot predicate over a graph that comes apart edge by
edge. In the N=3 rendezvous cell the three anchors were 13.18 / 34.18 /
33.98 s, so three robots holding a byte-identical pair kept it at times
spread over 21 s. The interval was exact and the appointment was not.

An instant removes the per-robot term entirely: the proposer computes it
once and everyone adopts the integer. What remains is the skew between
the robots' own mission-clock baselines, which is bounded by node start
and measured at ~1.2 s across the ts4 cells — against the 21 s spread it
replaces, and against the 30 s settle window it has to fit inside.
```

### `explo_planner_node.cpp:1249-1301`

**Documents:** `bool rendezvous_held_provisional_ = false;`

**Cites:** campaign, measured, `ts4`

```
WHY THE EXCEPTION EXISTS AT ALL, given that one-pair-per-meeting is
deliberate, and given that a provisional pair would be replaced at the
first meeting anyway. Because the first meeting may never come: the pair is
what the team drives to, so a placeholder is not merely a poor first
meeting point, it is a poor first meeting point that has to be kept before
anything can improve it.
The two clocks that have to line up for a tour-informed proposal are the
fleet becoming mutually complete and the allocator producing a tour, and
the ts4 smoke measured them both landing around t+25 s with the team
already dispersing: at N=4 the mutual window was about two seconds wide and
the tours were empty for all of it. Freezing the first thing derivable
makes the meeting place a function of which of those two won a race, which
is not a property anything should depend on.

"ONCE, EARLY, WHILE MUTUAL" WAS NOT A BOUND ON THE WINDOW (generation 22).
It reads like one and it is not, because `team_mutual` is a claim about the
last few seconds: the proposer may hold it true for a peer that has already
stopped listening. That is not a corner case, it is what happened — the
upgrade in ts4 smoke20's N=3 hybrid cell was authored 0.6 s after its last
peer went silent, and split the fleet across exactly the two generations
this paragraph claims to have narrowed. "Early" was doing the real work
here and it was never enforced.
```

### `explo_planner_node.cpp:1362-1379`

**Documents:** `bool                 rendezvous_reagree_waiting_ = false;`

**Cites:** measured

```
NO CLOCK OF ITS OWN. The wait is measured off the settle's clock, as
`settled_sec - rendezvous_settle_sec_`, because the two stages are
consecutive halves of one uninterrupted stand on the cell and a second
rclcpp::Time here would be a second thing to keep in step for no extra
information. It inherits the settle clock's property along with its
reading: node time, so an unresolvable mission clock cannot skip the wait.
```

### `explo_planner_node.cpp:1418-1476`

**Documents:** `std::vector<RendezvousProposal> rendezvous_peer_confirmed_;`

**Cites:** measured

```
WHAT THE OLD RULE COST. It required every peer to be inside
coord_claim_ttl_sec (5 s) AND holding the pair AND mutually in direct
contact, ALL AT THE SAME INSTANT. That is an N-way coincidence on an
occlusion-gated radio, and its probability collapses with team size: one
peer to line up at N=2, two at N=3, three at N=4. Measured on the gen-12
N=3 smoke — the proposer derived cell 24 at t=63.6 s and both followers
echoed it (t=67.1 and t=74.0), so all three robots genuinely held the same
pair — and the commit never fired because the 6.9 s between the two echoes
outlived the window. Every one of the 18 armings in that run refused with
"no (cell, interval) pair was agreed" while the fleet was, in fact, agreed.
```

### `explo_planner_node.cpp:1496-1525`

**Documents:** `double rendezvous_interval_sec_ = 300.0;`

**Cites:** campaign, `ts4`

```
THE SPACING OF THE TIMETABLE: the shortest gap between two legal meeting
instants, feeding RendezvousScheduler::Config::min_interval_ms. The
scheduler may only ever space the occurrences FURTHER apart than this (the
reachability floor), never closer, so this is the floor on the recurrence
and in practice is the recurrence — the tour term it competes with was
28.3 s at N=2 in the ts4 smoke.
```

### `explo_planner_node.cpp:1546-1569`

**Documents:** `bool rendezvous_anchor_starved_ = false;`

**Cites:** measured

```
It is a latch of the last observed mutual-contact instant, sampled on the
heartbeat timer — and that timer shares a single-threaded executor with the
state machine and the metrics sampler, both of which are measured to
overrun (hb_late_count_). A gap that swallows the whole mutual ->
not-mutual transition leaves the latch at the last tick before the gap
instead of at the separation, and every other robot, unblocked, holds the
right instant. That is the one failure this subsystem cannot absorb: the
pair is exact, the intervals match, and the two ends keep the identical
appointment at times that differ by the length of the stall.
```

### `explo_planner_node.cpp:1829-1831`

**Documents:** `double goal_rotate_timeout_sec_;`

**Cites:** measured

```
Deadline for the post-arrival in-place rotation, measured from the first
tick the robot is inside goal_xy_tol_ — NOT shared with nav_budget_sec_,
which budgets the drive (see doNavigate).
```

### `explo_planner_node.cpp:1883-1890`

**Documents:** `std::string done_criterion_{"latch"};`

**Cites:** banked

```
Which rule decides that THIS robot has finished exploring:
 - "latch" (default): the first tick on which the ROI unknown fraction
   reaches done_unknown_fraction, in ANY state and regardless of who is in
   comms range. Latched: it never un-finishes. No streak, no rendezvous.
 - "streak": the legacy rule — done_min_consecutive_steps consecutive PLAN
   ticks below threshold, then finishOrRendezvous, which may spend a
   reconnect manoeuvre before DONE. Kept so banked campaigns remain
   reproducible; see the param load for why the default changed.
```

### `explo_planner_node.cpp:1892-1898`

**Documents:** `std::string done_coverage_source_{"auto"};`

**Cites:** measured

```
Where the coverage-termination unknown fraction is measured:
 - "planning_map": 2D unknown cells in the latched planning_map (legacy).
   Returns -1 (check INACTIVE) when no planning_map is published.
 - "scovox": 2.5D column coverage of the ROI footprint on the fused 3D
   map in map_cache_ (MapCache::unknownColumnFraction) — works with no
   planning_map at all.
 - "auto" (default): planning_map when one has been received, else scovox.
```

### `explo_planner_node.cpp:2012-2041`

**Documents:** `double pursuit_speed_measured_mps_ = 0.40;`

**Cites:** measured

```
P1 / §3.8. The speed the robot ACTUALLY travels at (m/s), measured, used
for one thing only: deciding whether a chase is feasible at all.

It exists because the nav family's speed is a WATCHDOG estimate and the
pursuit gate was asking it a PREDICTOR's question. nav_speed_estimate_mps
(0.15) with nav_safety_factor (3.0) prices travel at 20 s/m, against a
measured 0.397 m/s — a factor of eight. For the watchdog that inflation is
harmless in the safe direction: budgets come out generous and a slow leg
gets more rope. For a REFUSAL it inverts. A bigger model time means "this
chase cannot finish", so the same conservatism that makes the watchdog
lenient makes the gate harsh: under the campaign cap of 600 s the gate
refused every chase past 600/20 = 30 m, which is exactly the observed
refusal floor (108 refusals, min 30.0 m, p50 42.9 m, max 75.1 m) in a plot
whose diagonal is larger than that. Pursuit was being declined on an
arithmetic artifact, not on an affordability judgement.
```

### `explo_planner_node.cpp:2043-2053`

**Documents:** `double pursuit_staleness_max_sec_ = 900.0;`

**Cites:** measured

```
Sized to real outages, not to goal freshness: dense-forest separations of
186-861 s were measured, and at the old 180 s the chase declined every
single time it was asked — pursuit was dead code in the world it was
written for. What makes the wider window safe is pursuit_goal_stale_sec
below: past 180 s the chase stops trusting the peer's declared goal and
drives to its last CONTACT POSE, a target whose value does not decay with
age, and declines outright when the budget cannot cover that trail.
```

### `explo_planner_node.cpp:2066-2085`

**Documents:** `bool pursuit_explore_fallback_ = true;`

**Cites:** campaign, count, measured, `p7modes`

```
Parking is a fixed point. Two robots that both hold cannot reconnect —
neither is moving, so the geometry that broke the link never changes —
and p7modes measured exactly that: 5 of 6 holds never reconnected, the
single recovery came from the PEER still driving, and one mutual hold
cost a mission whose maps were complete but split. A robot that goes
back to exploring is still covering ground, still earning the mission's
objective, and can regain the link by luck; a parked one can only be
found. Strictly dominated, so pursuit stops doing it.
```

### `explo_planner_node.cpp:2089-2107`

**Documents:** `double reconnect_confirm_sec_ = 3.0;`

**Cites:** campaign, count, measured, `p3b`, `p4mild`

```
Without this the arm test (peer missing, one read of the claim table) and
the release test (peer live, the same read one tick later) disagree inside
a single cycle, and the manoeuvre fires and dissolves before the robot
moves. Measured across the p3b/p4mild campaigns: 8 of 24 firings ended
within 5 s having travelled under a metre, two of them logging "ended
after 0.0 s sim", and 9 of 24 armed while the emulator had the pair
connected with 5-12 messages/s flowing. The cause is that the decision is
taken on a claim table which has not absorbed already-delivered intents:
the planner has just spent a long tick in PLAN (which is also what
suppresses its OWN beacon, see heartbeatTick), and drains the queue
immediately afterwards.
```

### `explo_planner_node.cpp:2116-2199`

**Documents:** `double reconnect_midrun_silence_sec_  = 90.0;`

**Cites:** banked, campaign, count, measured, ratio, stat, `g8r1`

```
--- Mid-exploration reconnect trigger ---
0 (the field default) keeps the manoeuvre strictly terminal: a robot only
considers reconnection once its own exploration is exhausted. A positive
value arms the same dispatch DURING exploration, whenever the team has
been continuously incomplete for this long — the point being that a
mid-run reconnection delivers the peer's queued map deltas while they can
still prune this robot's remaining frontiers. Must exceed the worst
heartbeat-suppression episode (measured ~180 s in the sim campaigns, see
heartbeatTick): below that, a healthy teammate stuck in a long PLAN loop
reads as missing and the trigger drives a manoeuvre at a robot that is in
range and fine. 0 restores the legacy terminal-only trigger.

Default 90 since generation 9. It was 240 through generation 8, chosen to
clear that 180 s suppression tail by pure waiting because the record-age
clock could not tell a silent teammate from an absent one. The link veto
below now makes that distinction directly from the radio, so the threshold
no longer has to be set by the suppression tail — and 240 was measured to
be far out in the tail of the outage distribution this binary actually
produces. In g8r1's 23 hybrid cells the clock expired in 3, so 87 % of the
treated arm ran behaviourally identical to the control (§32.15).

Second draft: 90, re-scored on two clocks but on the 20 g8r1 HYBRID cells
that never dispatched. That population is SELECTED ON THE OUTCOME the sweep
varies, so its 240 row read "0/20 arm" by construction -- a definition
printed as a measurement -- and the monotonicity below it was mostly the
selection. The off arm never ran the trigger at all and was sitting there
unused: 23 untreated cells, no contamination, no selection.

Scored on the OFF arm's banked link_states.csv, with the veto as it ships
(reconnect_link_down_confirm_sec = 0, i.e. "not up right now"):
    presence   cells arm   fires   wasted   beats natural recovery
       240        6/23       12      50 %           33 %
       150       12/23       24      42 %           58 %
       120       18/23       36      33 %           72 %
        90       18/23       40      20 %           85 %
        75       18/23       42      29 %           86 %
        60       21/23       60      43 %           73 %
"wasted" = the radio outage in progress ended within the ~14.6 s it takes
to start moving. "beats natural recovery" = the fire happened more than
one chase (~53 s, p13 median) before the radio next came up and STAYED up
for 30 s, i.e. the manoeuvre had something real to buy.

On THIS grid -- unwalked -- 90 is the argmin of wasted and within a point
of the max of beats, and 120, 90 and 75 all arm the same 18/23 cells.

THE MODEL IS OPTIMISTIC AND HERE IS BY HOW MUCH, AND THE GRID DOES NOT
SURVIVE IT. The table evaluates the veto at the instant the presence clock
crosses T. The binary evaluates on PLAN ticks, so coord_claim_ttl_sec
(4.98-4.99 s measured) plus tick granularity (9.02-13.82 s measured) puts a
real fire 14.0-18.8 s later, in which window a link can change state either
way. Scored at T + that overshoot, at each of the three measured offsets,
the waste argmin moves off 90 and onto 75 (+14.0: 20.0 % vs 90's 28.6 %;
+16.4: 19.2 % vs 26.3 %; +18.8: 25.0 % vs 29.4 %) and the plateau breaks
(120 -> 14/23, 15/23, 17/23 against 90's 18, 19, 17). An earlier draft of
this comment said the walk "moves the hybrid-arm count 11/20 -> 12/20" and
left the direction of the table unaffected; BOTH halves were wrong. The
unwalked hybrid value is already 12/20 and walking gives 12, 10, 11 -- an
overshoot cannot buy activation -- and the table does reorder.

90 IS THEREFORE NOT CHOSEN BY THIS GRID. A ranking that flips under a 4.8 s
change in a nuisance offset is one 23 cells cannot resolve, and re-tuning to
75 on the same 23 cells would repeat the error the grid was already
criticised for. What survives the walk is only the coarse verdict -- 240 far
too high (4-6 of 23 at every offset), 60 past the point where waste turns
back up -- and inside 120-75 the choice is made off the grid: 90 clears the
p90 radio outage (52.0-111.0 s, nearest-rank, 13 tags) and sits far below
the 240 s point where the presence clock stops filtering at all.

Out-of-sample check: at generation 8's ACTUAL configuration (T = 240,
confirm = 3) the model arms 4 of 23 off-arm cells, against the 3 of 23
hybrid cells that really dispatched. Close, not exact, and quoted that way.

WHAT THIS DOES NOT CLAIM. The rate is fitted on generation-8 traces and
generation 9 changes behaviour, so 18/23 (~78 %) is an estimate of the
treated fraction, not a prediction, against generation 8's measured 3/23.
The analysis must carry the dilution rather than assume it away: the
between-arm effect is intention-to-treat over a partly-treated hybrid arm.
```

### `explo_planner_node.cpp:2211-2257`

**Documents:** `double rendezvous_depart_delay_sec_ = 100.0;`

**Cites:** campaign, count, `ts4`

```
  the separation anchor  gen 17 keyed the occurrence off it; the ts4 N=3
                         cell anchored its three robots 13.2 / 34.2 /
                         34.0 s apart, so gen 18 removed it.
  each robot's own now   gen 18's replacement; the same cell armed at
                         16.1 / 52.5 / 67.4 s — a 51 s spread against a
                         30 s period, so the three robots walked to three
                         DIFFERENT meetings (t+34, t+64, t+94).

The place is still agreed in advance by the whole team, which is the half
of the protocol that was never broken — the ts4 N=3 cell committed a
byte-identical triple on all three robots within 21 ms.

INERT SINCE GENERATION 25. Through generation 24 this was a per-robot
notice floor on the arming (attend the first occurrence at or after
now + delay), and that per-robot term is what forked the ts4 N=3 cell:
the 33 s arming spread plus 100 s of notice straddled the agreed instant,
two robots kept it and the third rolled a whole interval past it. The
arming floor is now bare `t_now`. The parameter stays declared, read and
logged ONLY so the manifest schema and the cross-arm param rows stay
comparable with earlier generations; it is not validated (see the
validation block) and it decides nothing. Flap patience lives where it
always really was: the reconnect
silence window gates the arming, and the P5 supersede cancels a standing
appointment when the team returns before departure.
```

### `explo_planner_node.cpp:2260-2275`

**Documents:** `double rendezvous_appointment_wait_sec_ = 0.0;`

**Cites:** campaign, count, `ts4`

```
SEPARATE FROM reconnect_midrun_max_wait_sec_ ON PURPOSE. That cap is what
bounds a PURSUIT — a chase to a predicted intercept that may be at the
wrong place entirely, where giving up is the correct response to a bad
prediction. An appointment is the opposite: the place is one every robot
agreed to, and the only reason a peer is not there yet is that it has
further to drive or noticed later. Giving up on it converts a meeting that
was going to happen into a no-show, which is exactly what the ts4 N=3 cell
did — husky burned the full 240 s and walked away from a cell two of three
robots had already stood on together.
```

### `explo_planner_node.cpp:2278-2305`

**Documents:** `double rendezvous_max_lateness_sec_ = 60.0;`

**Cites:** measured

```
How late a robot may agree to BE. The team's schedule is a lattice —
t_meet + k*interval — and this decides which rung THIS robot signs up to:
the first one it can still reach with no more than this much lateness,
measured at arming against its own marked-up drive (appointmentLeadMs).

WHY NOT ZERO. The lattice spacing is floored at the furthest robot's drive,
so a zero budget would roll a robot a whole interval for being a second
short — paying one full period of separation to avoid one second of
waiting, when the barrier at the far end absorbs waiting for free. 60 s is
Kalhan's figure (2026-09-19); against the 300 s lattice it rolls about one
arming in five, and it is what the chase window above is measured against.
```

### `explo_planner_node.cpp:2353-2379`

**Documents:** `int    reconnect_midrun_max_attempts_ = 6;`

**Cites:** banked, campaign, count, measured, `ts4`

```
THAT IS A DOSE TERM, AND IT IS NOT UNIFORM ACROSS THE DESIGN, which is why
it is spelled out here rather than left to the reader:
  * By ARM. Exhaustion removes mid-run chasing, so it bites `pursuit` and
    hybrid's chase half. It barely touches `rendezvous`, whose trigger is
    already suppressed whenever an appointment stands, and not at all in
    `off`.
  * By RUNG. More robots means more outages means the budget is reached
    sooner in mission-elapsed terms. At the smoke's measured mesh uptimes
    (N=3 rendezvous 29%, N=4 14.7%) the N=4 rung spends attempts fastest.
So a run that exhausts the budget is running a WEAKER treatment from that
point on, and the point arrives earlier at N=4 than at N=2. Whether the
budget should refund successes is a treatment-design question for Kalhan,
NOT an arithmetic defect, so it is left alone and made observable instead:
`midrun_attempts_used` on run_end says how close each cell came, which is
what an analyst needs to decide whether the cap bound anything at all. In
the banked smoke cells it never exceeded 1 of 6, but those cells are ~300 s
against ts4's 3000 s.
```

### `explo_planner_node.cpp:2381-2412`

**Documents:** `std::string comms_link_states_topic_;`

**Cites:** measured, stat

```
--- Link-state gating for the mid-run trigger (see §30.11 and §30.24) ---
THE DEFECT THIS FIXES. Everything above measures silence with a RECORD-AGE
clock: team_last_complete_time_ advances only while peer intents arrive,
and the intent beacon is conditional twice over (it needs an active intent
and a state outside PLAN), so a healthy in-range teammate stuck in a long
PLAN loop is indistinguishable from one behind a hill. Measured against the
emulator's own link trace the record clock ran a median +49.1 s ahead of
real link-down, and 41-42 % of all mid-run fires bought nothing — 16-21 %
of them fired while the radio was UP. A chase is real distance debited from
exploration, so those are pure loss.
```

### `explo_planner_node.cpp:2419-2455`

**Documents:** `double reconnect_link_down_confirm_sec_ = 0.0;`

**Cites:** campaign, count, measured, stat, `g8r1`

```
0 IS THE DEFAULT, AND THE 30 s DEBOUNCE THAT WAS HERE IS WITHDRAWN.
30 was chosen off the outage distribution (above the 6.2-12.2 s median
flicker, below the 52.0-111.0 s p90 -- nearest-rank over the 13 tags with
>= 10 cells; linear interpolation would read 51.4-101.6) and credited in an
earlier draft of this
comment with moving wasted fires "from 50 % to 33 %". That was
misattributed: the table it pointed at held this conjunct FIXED at 30 and
varied the presence clock, so 50->33 is the 150->90 move, not this
parameter's effect. Measured
properly, with the presence clock pinned at 90 and only this varying:
    confirm   cells arm (off arm)   fires   wasted   beats
       0            18/23            40     20 %     85 %
      30            12/23            26     15 %     92 %
The debounce buys 5 points of fire purity for A THIRD of the arm's
activation: 18 arming cells down to 12, i.e. 6 of the 18 that armed. (An
earlier draft said "a QUARTER" -- that is 6/23, the share of the arm; the
denominator the purity is traded against is the 18 that armed, not the 23
that exist. The six-cell loss the paragraph below already costs out is the
same six, so only the fraction was wrong, not the trade.)
Dilution is the defect generation 9 exists to correct -- 87 %
of g8r1's treated arm was behaviourally the control -- so trading
activation for purity spends the fix on a refinement. The four extra
wasted chases it would prevent cost ~53 s each against run times in the
hundreds to thousands of seconds; the six lost treated cells cost power
that no amount of analysis recovers.
```

### `explo_planner_node.cpp:2504-2518`

**Documents:** `rclcpp::Time link_up_last_seen_;`

**Cites:** measured

```
Receipt time at which the link was last OBSERVED up. Down-duration is
measured from here rather than from a stored up->down edge, because
Float64MultiArray carries no header: if the executor stalls in a long PLAN
tick, a backlog delivered afterwards is all stamped at receipt, which would
compress the history and read the down-duration as ~0.
```

### `explo_planner_node.cpp:2520-2540`

**Documents:** `double reconnect_min_share_voxels_     = 0.0;`

**Cites:** campaign, count, measured, stat, `p7modes`

```
--- Info-gated mid-run trigger (voxels, not seconds) ---
0 (the default) keeps the fixed silence clock above, bit-identical legacy
behaviour. Positive: the trigger time becomes "when the pair's estimated
unshared map crosses this many voxels", dead-reckoned from the last
contact: T = min_share / (my_rate + peer_rate), both rates frozen in the
LastContact snapshot so the two robots derive (approximately) the SAME
trigger time with no in-outage communication — asymmetric firing is the
lone-waiter failure p7modes measured (5 of 6 holds never reconnected).
T is clamped to [min_silence, max_silence]; rates that read 0 (peer
predates the beacon field, or no samples yet) push T to max_silence, i.e.
"nothing known to share -> wait for the backstop", never a divide-by-0.

Calibrated against p14 (36 comms-on cells, dense world): a chance merge
delivers ~16k voxels median, a commanded one ~332k; by the 240 s clock
the pair is ALWAYS 300-850k apart — so as a deferral filter this gate is
inert in that world, and its live use is EARLIER, timeliness-driven
triggering. Sizing follows from the measured pair divergence (~2.3k
vox/s under this estimator): T ~ V*/2300, so 200k ~ 84 s, 300k ~ 126 s,
500k ~ 210 s. Anything below ~140k lands under the 60 s floor and the
clamp — not V* — becomes the trigger, which is a fixed clock wearing the
gate's name; pick V* above that or the arm tests nothing.
```

### `explo_planner_node.cpp:2542-2570`

**Documents:** `double reconnect_midrun_min_silence_sec_ = 60.0;`

**Cites:** stat

```
Clamp bounds for the derived trigger time. The floor guards the radio-
flicker regime the release-confirm machinery expects (a healthy teammate
in a long PLAN loop can read missing ~180 s; an info trigger BELOW that
knowingly accepts some chases at busy-not-lost teammates — that is the
eager variant's cost, so the floor is a parameter and not 200 hardcoded).
The ceiling is the insurance policy: the ONE effect p14 proved is that
mid-run reconnection caps the worst outage (515 -> 308 s, p=0.0095), and
no info gate is allowed to trade that away by deferring forever. It
therefore defaults to the LEGACY CLOCK, not above it: the gated arm can
then only ever fire earlier than the control, so the proven cap is a floor
on its behaviour and the comparison carries no "gated runs waited longer"
confound. A ceiling above reconnect_midrun_silence_sec is a deliberate
choice to give that up. Because it is defined as TRACKING that clock, the
initialiser below is only a mirror for readers: the value that actually
takes effect is defaulted to the resolved reconnect_midrun_silence_sec at
the parameter-read site, so the two cannot drift apart the way they did
when generation 9 moved the clock and this line was hand-copied after it.
THE FLOOR NOW BINDS OVER MOST OF THE USABLE RANGE, which it did not when
the ceiling was 240. With the ceiling tracking 90, T = V*/2300 is pinned to
the ceiling for anything above ~207k voxels and to the 60 s floor below
~138k, so the window in which V* -- rather than a clamp -- actually decides
is roughly [138k, 207k]. It was [138k, 552k] at a 240 s ceiling. The
sizing note above ("300k ~ 126 s, 500k ~ 210 s") describes the UNCLAMPED
derivation and both of those now clamp to 90. Anyone enabling the info
gate at generation-9 defaults must either pick V* inside that narrow
window or raise the ceiling deliberately, otherwise the warning that
comment gives -- a fixed clock wearing the gate's name -- is what they get.
Both are inert while reconnect_min_share_voxels is 0, which is the default,
so nothing in the shipping configuration depends on this.
```

### `explo_planner_node.cpp:2587-2602`

**Documents:** `bool   hold_escalate_          = true;`

**Cites:** count

```
--- Hold escalation (mutual-hold deadlock break) ---
When a TERMINAL barrier wait expires, drive once to the last-connected
anchor and wait hold_escalate_wait_sec more before giving up. Both robots
converging on their own last-contact poses restores the pair geometry the
link last worked at, which breaks the pure-hold fixed point (observed: 5
of 6 holds never reconnected; the parked pair can only be rescued by peer
motion). Sticky per-manoeuvre flag, NOT a position test: an unreachable
anchor must not re-escalate on every expiry forever. Inert under the
field default rendezvous_max_wait_sec=0 (wait forever, so no terminal
barrier ever expires): it can only act where an escape hatch is already
configured, and there it converts a give-up into one more attempt.
```

### `explo_planner_node.cpp:2629-2650`

**Documents:** `double rendezvous_present_tol_m_ = 10.0;`

**Cites:** campaign, seed, `ts4`

```
It exists because the two watchdogs in doReturnNav used to hand the barrier
an arbitrary pose. Their note argues a stopped robot is "at least closer to
comms than where exploration stranded us" — true of the anchor return it was
written for, and false of an appointment, where the OTHER robots are at the
cell and nowhere else. The ts4 gen-30 N=2 rendezvous seed8 cell priced it:
one robot reached the cell (1.3 m), its partner's drive died 40.0 m away and
joined the barrier from there, leaving the pair 41.3 m apart with three
trunks between them — past the 30 m horizon, so the presence each was
waiting on could never arrive. Both stood still for 2609 s and the run was
killed by the harness hang detector.
```

### `explo_planner_node.cpp:2795-2800`

**Documents:** `double exploit_dwell_sync_max_wait_sec_ = 0.0;`

**Cites:** measured

```
Barrier give-up (s), measured from EXPLOIT_DWELL entry and NOT from the
re-anchored dwell start. <= 0 (the default) waits until the peer actually
arrives; see the release-path argument in doExploitDwell for why that
terminates. Set a positive value only to force a deadline: a wall-clock
bound cannot distinguish a distant teammate from a wedged one, and 60 s
abandoned one that needed 111 s to cross the plot.
```

### `explo_planner_node.cpp:2849-2864`

**Documents:** `rclcpp::Time dwell_sync_wait_start_;`

**Cites:** measured

```
Dwell-sync barrier bookkeeping, re-initialised on every EXPLOIT_DWELL entry
(transitionTo). The wait clock CANNOT be state_enter_time_: the barrier
holds by re-anchoring that to now every tick, so a wait measured from it
would read ~0 forever and max_wait would never fire. The latch is what
stops a timed-out barrier from re-entering and re-anchoring the dwell clock
on the next tick, which would leave the dwell never completing.
```

### `explo_planner_node.cpp:2988-3003`

**Documents:** `bool   done_seek_enabled_       = false;`

**Cites:** banked, count

```
--- Post-latch coast (done_seek_enabled) ---
A latch that lands MID-MANOEUVRE currently cancels the chase: the robot
brakes, discards its frozen contact pair, and the partner keeps waiting at
a barrier for a robot that is no longer coming. That is not an edge case —
it is the modal way a manoeuvre ends, 56 of 88 reconnect_end events across
the banked campaigns and 58-76% within every hybrid arm.

OFF by default, and deliberately a RUNTIME switch rather than a build:
every banked campaign ran without it, and the A/B for it has to sit inside
ONE run_campaign.sh invocation or the arm is confounded with the session.
```

### `explo_planner_node.cpp:3036-3109`

**Documents:** `double mission_start_hold_sec_  = 60.0;`

**Cites:** banked, campaign, measured, `ts4`

```
WHY IT EXISTS. The protocol's one free window is the start of the run, when
the team is co-located and every link is up — the derive gate says so in as
many words, and the measurement backs it: the centroid placeholder commits
with a full echo round, on every robot, within about 21 ms. What was NOT
guaranteed is that the window lasts long enough to also carry the ONE
upgrade the proposer is allowed. It did not, in the ts4 smoke20 N=3 hybrid
cell: bestla's last peer went silent at t+65.1 s and atlas authored the
upgraded triple at t+65.7 s, 0.6 s later. bestla kept the 26-s-stale
placeholder, atlas and husky took the upgrade, and the fleet drove to two
different cells and waited out the rest of the run in two places. The team
had dispersed before it had finished agreeing.

WHAT THIS HOLD DOES NOT DO, because it shipped for one day claiming to.
A companion edit confined the provisional->final upgrade to this window, on
the theory that an upgrade authored inside it cannot split the fleet. The
theory is sound and the implementation was measured to be worthless: the
upgrade needs candidate cells, candidate cells come from the allocator's
tours, tours come from completed exploration steps, and a robot held here
completes none. Confining the upgrade to the hold does not schedule it
earlier, it prevents it forever. The full measurement is in the derive gate
in maintainRendezvousProposal; the conjunct is gone and must not come back.
What remains here is only the first half: agree the placeholder while
everyone is still standing together.

THIS NUMBER WAS 120 s FOR ONE DAY (2026-09-17) AND THE ARGUMENT FOR 120 IS
WITHDRAWN. It was: the window must also contain the provisional->final
upgrade, whose first occurrence was swept at 41-77 s over the banked cells,
so 60 s sat below the N=2 minimum of 61.6 s. Every word of that is true and
it is irrelevant, because the upgrade is no longer confined to the window —
confining it was measured to delete it rather than schedule it, since the
upgrade's input is completed exploration steps and a held robot completes
none. With the confinement gone the upgrade fires whenever the allocator
first has tours, at t=41 s or t=353 s, hold or no hold. The hold neither
helps nor hinders it, so the sweep no longer constrains this number at all.
```

### `explo_planner_node.cpp:3175-3181`

**Documents:** `std::vector<Eigen::Vector3f> home_trail_;`

**Cites:** ratio

```
Breadcrumb trail for the retrace fallback. The nav global planner never
plans (66/66 robot-logs, §28 of the experiment doc), so a long direct
home goal is greedy local navigation and can trap in a local minimum
(mr0pilot_hybrid_seed4 bestla: parked 31 m out). The trail is the
robot's own outbound positions at >=2 m spacing — ground it has already
traversed once — recorded from home capture until homing starts. The
second no-progress retry follows it back crumb by crumb.
```

### `explo_planner_node.cpp:3200-3203`

**Documents:** `bool         near_home_armed_ = false;`

**Cites:** measured

```
Band clock for the near-home stall detector. Armed on entry to the
suppression band and cleared on leaving it, on an escape leg, and on a
relocalization jump -- a jump can teleport the robot into the band, and
dwell time measured across one is not dwell time.
```

### `explo_planner_node.cpp:3221-3234`

**Documents:** `static constexpr float kEscapeBandMinM      = 1.5f;`

**Cites:** campaign, measured, stat, `ts1b`

```
That suppression used to claim the frozen detector and the nav budget still
covered the band. They do not, and the band is wider than the arrival test:
arrival needs <= mission_home_tol_m (1.0 m), the mute radius is 3.0 m, and
between the two a robot that keeps MOVING without closing the gap passes
the frozen test every window (it accumulates distance) while the approach
test is switched off. Only the overall nav budget remained, minutes away.
Measured on ts1b: 7 homings parked in that band for 221-565 s against a
70.9 s median homing, 6 of which set their cell's makespan. The band clock
above closes it -- see doReturnHome.
```

### `explo_planner_node.cpp:3250-3252`

**Documents:** `bool pose_jump_seen_ = false;`

**Cites:** measured

```
Raised by trackDistance's teleport guard, consumed by doReturnHome: an
approach delta measured across a relocalization discontinuity is a
measurement artefact, not a stall.
```

### `explo_planner_node.cpp:3375-3379`

**Documents:** `int   pending_plan_cand_total_    = -1;`

**Cites:** measured

```
Full rejection profile of the most recent doPlan attempt. Unlike the two
above these are drained by fillCommonMetrics, so they reach EVERY row
including the timer rows a starved planner emits — which are the only rows
it emits, since a tick that selects nothing never completes a step. See
metrics_logger.hpp. -1 = no planning attempt yet, never a measured zero.
```

### `explo_planner_node.cpp:3430-3445`

**Documents:** `static constexpr double kRateWindowSec = 300.0;`

**Cites:** measured, stat

```
The window is 300 s, NOT the contact duration: connected windows have a
median of 24 s and 72% are under 90 s (measured, p14), so a window sized
to a contact contains nothing but that contact's merge inflow. Reaching
back across the PRECEDING OUTAGE is what supplies samples of the robot's
own unaided gathering. Validated against p14's measured pair divergence
(2,134 vox/s): the pair's summed rate under this estimator reads 1.12x
truth, against 1.40x for a 90 s two-point slope, at better coverage.
```

### `explo_planner_node.cpp:3573-3579`

**Documents:** `int    metrics_rows_written_ = 0;`

**Cites:** measured

```
REALISED sampling accounting for the periodic CSV sampler. The configured
period is not what it achieves: the sim-time timer above is gated by the
steady-clock deadline in metricsTick, so at some real-time factors whole
ticks are suppressed and at others none are — a realised period of 10 s
against a configured 5 s was measured in an earlier campaign, with every
log line still asserting 5 s. Counted here and reported in run_end so the
rate of an archived run is a recorded fact.
```

### `explo_planner_node.cpp:3598-3605`

**Documents:** `int mission_complete_count_ = 0;`

**Cites:** measured

```
mission_complete emissions this run. The contract is exactly one, and TWO
guards at the top of startReturnHome enforce it: mission_return_done_
(run-scoped) and the state_ == RETURN_HOME test (leg-scoped). Through
generation 8 this comment named only the second, which structurally cannot
see the defect that was actually measured — the second request arrives
from DONE, not from RETURN_HOME, so the state test never fired on it.
Incremented on every emission unconditionally: a counter that a guard
prevents from ever reaching 2 would be a check that stopped checking.
```

### `explo_planner_node.cpp:3607-3620`

**Documents:** `bool mission_return_done_ = false;`

**Cites:** campaign, count, measured, `ts1b`

```
Why the state test cannot carry this: finishMissionReturn ends in
finishNow, so the robot is in DONE — not RETURN_HOME — by the time the
second request arrives, and it does arrive. The coverage latch fires from
the metrics tick, which is the deciding hook for done_criterion=latch and
measures the ROI in EVERY state, so a robot that ended on barrier-gave-up
can saturate its map minutes later while parked at home and ask to go
home again. Measured on ts1b: 16 DONE->RETURN_HOME re-entries and 16
robot-runs carrying two mission_complete rows, all of them in the
treatment arm, the count rising with team size.
```

# Constructor — parameter declaration and validation

### `explo_planner_node.cpp:3722-3749`

**Documents:** `coverage_milestones_ = dp("coverage_milestones",`

**Cites:** campaign, measured, `p8trigger`

```
The default is sized from the campaign data rather than guessed. Across 98
planner CSVs under /tmp/hmr_campaign the first measured unknown fraction is
0.930-1.000 and the final one is 0.497-0.554 in 97 of them (the exception
is a 21-row run aborted at 0.817). So: 0.95 is degenerate — for the robots
whose first sample is already 0.93 it would fire at t0 by construction and
measure the initial map, not exploration — and nothing below 0.50 is ever
reached. 0.90 down to 0.50 is the informative band (in the p8trigger runs
0.90 lands at ~35-43 s and 0.55 at ~820-1490 s, a spread that separates the
arms), and 0.45 is carried as a guard rung so a future arm that covers more
ground still records the crossing instead of silently having no data point.
Rungs that never fire simply never appear in the file.
Ladder rationale (and why the tail is NOT 0.55/0.50/0.45) in
config/shared_params.yaml — measured, those bottom rungs were dead columns
and the deepest one that fired was done_unknown_fraction itself.
```

### `explo_planner_node.cpp:3752-3759`

**Documents:** `experiment_log_anchor_period_sec_ =`

**Cites:** measured

```
clock_anchor cadence in SIM seconds. Each anchor is a (sim, wall) pair plus
the real-time factor since the previous one, which turns this file into the
conversion table for every other log in the run directory — they carry wall
stamps only, and the current practice of fitting one line through the whole
run is wrong by 10-54 s because the RTF drifts within a run (0.89 -> 0.81
measured). 10 s bounds the interpolation error to well under a second at
any plausible drift rate and costs one short line per anchor. <= 0
disables anchors (every other event still carries its own pair).
```

### `explo_planner_node.cpp:3788-3804`

**Documents:** `if (goal_yaw_tol_ < 0.3) {`

**Cites:** campaign, count, measured, stat, `ts1b`

```
The navigator is simple_nav_3d, not nav2 — this comment used to justify the
0.3 threshold with nav2's general_goal_checker defaults (xy 0.25 /
yaw 0.25), a checker that has never run in this system. simple_nav_3d's UGV
stop condition is ugv.goal_xy_tol_m / ugv.goal_yaw_tol_rad, both 0.2 as
shipped, but the park distribution is set by four stacked roundings rather
than by either tolerance and the measured median arrival over ts1b's 240
cells landed 0.269 m out. 0.3 is therefore the point below which the gate
stops clearing the measured median, which is what these two warnings are
actually about. A node cannot read another node's parameters, so the
tolerance figures quoted below are documentation and have to be checked by
hand against simple_nav_3d/launch/simple_nav_3d.launch.py; the 0.269 m is
the load-bearing number and it is independent of them.
```

### `explo_planner_node.cpp:3820-3848`

**Documents:** `cand_min_goal_dist_ = dp("candidate_min_goal_dist_m", 0.0);`

**Cites:** measured

```
Without it the planner deadlocks into a two-point oscillation, and the
mechanism is structural rather than a tuning accident. Utility is the
SSMI-style info_gain / (eps + path_cost). In a mostly-unknown map every
candidate's raycast terminates in unknown space, so info_gain is nearly
constant across the whole candidate set — measured spread was 6% while
path_cost varied ninefold — and argmax(U) degenerates to argmin(cost),
i.e. "drive to the closest frontier". The closest frontier is typically
under a metre away, and here is why that never resolves: a VLP-16 has a
+-15 deg vertical field of view, so at 0.6 m of standoff it sees a band
barely 0.3 m tall. Voxels beside the robot at any other height are
physically unobservable from that range. Driving there reveals nothing,
the frontier survives, and the pair of candidates either side of the
robot regenerate every step forever. Neither guard already in the loop
catches it: goal_xy_tolerance only skips candidates at arm's length, and
the failed-goal blacklist never fires because the robot REACHES each goal.
```

### `explo_planner_node.cpp:3886-3901`

**Documents:** `failed_goal_radius_m_ =`

**Cites:** measured

```
The TTL must outlast one full worst-case attempt SOMEWHERE ELSE plus the
travel and replanning around it. It did not: the 60 s default was sized to
an old 60 s budget cap, and when nav_max_timeout_sec went to 180 s in the
campaign yaml the TTL was not raised with it. The consequence, measured in
mr1_hybrid_seed18: each of two terrain traps expired from the blacklist
while the robot was busy burning a 180 s budget at the other one. Observed
fail -> re-pick gaps for the same trap were 210, 203, 224 and 225 s — all
longer than 60 and all shorter than 240. Eight full budgets, 1441 s, two
sites, and the cell ended 0.021 of unknown-fraction short of done.
```

### `explo_planner_node.cpp:3906-3924`

**Documents:** `failed_goal_retire_after_ = dp("failed_goal_retire_after", 3);`

**Cites:** count

```
k=3 from the mr1 evidence: across 72 robot-logs and 49 distinct failure
sites, reattempts at [budget] sites succeeded 0/7, and exactly 1 of 49
sites was later reached within the 2 m radius — a no-progress site that
recovered on its FIRST reattempt, which k=3 would not have blocked.
```

### `explo_planner_node.cpp:3927-3930`

**Documents:** `if (failed_goal_ttl_sec_ < nav_max_timeout_sec_ + 30.0) {`

**Cites:** seed

```
Make the drift that caused seed18 impossible to repeat silently. The two
numbers are coupled — a blacklist entry has to outlive one attempt
elsewhere — and nothing enforced that coupling, so raising one of them in
the campaign yaml quietly disarmed the other.
```

### `explo_planner_node.cpp:3939-3960`

**Documents:** `visited_goal_radius_m_ = dp("visited_goal_radius_m", 0.0);`

**Cites:** measured

```
Needed because frontier exploration has no fixed point in a forest. A
frontier is a free voxel beside an unknown one, and every trunk casts a
permanently unknown shadow, so frontier clusters regenerate no matter how
thoroughly an area is observed. Utility is info/(eps+cost) and info is
near-constant while the map is mostly unknown, so selection collapses to
argmin(cost) -- and two neighbouring clusters then trade places as
"nearest" forever. Measured on flatforest: after narrowing the frontier
band the robot advanced in bursts but still alternated between two goals
4.7 m apart for six consecutive steps with no net movement.
```

### `explo_planner_node.cpp:4009-4024`

**Documents:** `done_criterion_ = dp("done_criterion", std::string("latch"));`

**Cites:** campaign, measured, `tr1`

```
WHICH RULE DECIDES FINISHED. The default is "latch" and the paragraph above
describes "streak", which is now opt-in. The change is deliberate and the
reason is measurable: the streak test runs only at the top of doPlan, so a
robot inside a reconnect manoeuvre cannot declare itself finished however
saturated its map is. Measured on the tr1 campaign, that blind spot plus
re-earning the streak from zero after the manoeuvre put ~60 s of pure
detection latency on the hybrid arm and 0 s on the off arm — an endpoint
that moves with the treatment is not an endpoint, it is part of the
treatment. "latch" measures the same quantity in both arms:
  * the robot's OWN fused-map ROI unknown fraction (nothing team-wide),
  * tested on every metrics tick in EVERY state, manoeuvres included,
  * on first touch — no confirmation streak,
  * with no rendezvous gate: being in comms is not required to be finished.
The run then ends when BOTH robots have latched independently, which the
harness already implements by waiting for every planner's state to read
DONE. Set "streak" to reproduce a pre-2026-08-24 campaign.
```

### `explo_planner_node.cpp:4137-4162`

**Documents:** `const double f_lo_off = dp("frontier_z_lo_offset_m", 0.0);`

**Cites:** measured

```
The ROI band is sized for terrain and canopy — the yaml ships a 9.5 m slab,
-5.5 to +4.0 — and searching a slab that tall for frontiers produces a
candidate set dominated by voxels no robot can ever observe. A frontier is
a free voxel with an unknown neighbour, and a lidar's free space is a wedge
bounded by its vertical FOV, so the ENTIRE upper and lower surface of that
wedge qualifies, at every range, forever: a VLP-16 at +-15 deg simply has
no ray that reaches 3 m up at 4 m out. Those frontiers cannot be consumed
by driving to them, which is what makes them poison rather than noise —
they regenerate beside the robot every tick, they are always the nearest
ones, and (utility being info/cost with near-constant info) they are
therefore always chosen. Measured on flatforest: frontier_voxels stayed at
~94% of all observed voxels and GREW monotonically as the map grew, while
both robots ping-ponged between two adjacent goals indefinitely.
```

### `explo_planner_node.cpp:4249-4261`

**Documents:** `{`

**Cites:** stat

```
Defaults are OFF (weight 0), so this block changes no shipped behaviour
until a campaign turns it on. The radius default is the 20 m the
rho = 0.677 correlation is stated at, and the max age is two TeamWorld
heartbeats plus margin; both are inert while the weight is 0.
```

### `explo_planner_node.cpp:4354-4385`

**Documents:** `{`

**Cites:** campaign, `cr3`, `cr5`

```
Renamed from `rendezvous_enabled` on 2026-09-03. The old name predates
pursuit and hybrid existing, and by cr3-cr5 it read off the manifest as
"the rendezvous arm is on" in three arms out of four — including pure
pursuit, which cannot arm an appointment at all (armAppointment refuses on
mode). It cost a reader a double-take on a finished campaign; that is the
whole reason for the rename.
```

### `explo_planner_node.cpp:4637-4638`

**Documents:** `done_seek_enabled_ = dp("done_seek_enabled", false);`

**Cites:** banked

```
Post-latch coast (see the member comments). OFF by default so this binary
reproduces every banked campaign bit-for-bit on the control side.
```

### `explo_planner_node.cpp:4658-4660`

**Documents:** `mission_return_enabled_ = dp("mission_return_enabled", false);`

**Cites:** banked

```
Mission return (see the member comments). OFF by default for the same
reason as the coast: this binary must reproduce banked behaviour exactly
unless the campaign explicitly opts in.
```

### `explo_planner_node.cpp:4664-4669`

**Documents:** `mission_start_hold_sec_ = dp("mission_start_hold_sec", 60.0);`

**Cites:** banked

```
The pre-mission hold (see the member). Defaulted ON, unlike the two knobs
above, and the asymmetry is the point: those change what the robot does and
must be opted into so the binary reproduces banked behaviour, whereas this
one closes a defect that split a fleet. A campaign that forgets the knob
should get the fix, not the split. Generation 22 is a new generation
precisely because of it and is not poolable with anything earlier.
```

### `explo_planner_node.cpp:4682-4689`

**Documents:** `return_approach_window_sec_ = dp("return_approach_window_sec", 40.0);`

**Cites:** seed, stat

```
Approach watchdog (generation 5). 1.0 m per 40 s = 0.025 m/s net, which is
~9x below the slowest ARRIVED homing in the 71-homing population (mean
approach 0.220 m/s, median 0.388) and infinitely above seed11's -0.0001
m/s, so the separation is not marginal in either direction. The 40 s
window is long enough for a legitimate circumnavigation (worst arrived
detour ratio 1.47) and short enough that two fires still leave 5/6 of the
600 s cap. Escapes are capped so the whole ladder is bounded well inside
the cap: 40 + 40 + 3 x (30 escape + 40 window) = 290 s worst case.
```

### `explo_planner_node.cpp:5020-5034`

**Documents:** `#ifdef EXPLO_PLANNER_GIT_REV`

**Cites:** campaign, `cr5`

```
EXPLO_PLANNER_GIT_REV comes from explo_planner_git_rev.h, regenerated on
every build (cmake/StampGitRev.cmake), so it is the revision this binary
was actually compiled from. It was previously a configure-time compile
definition and went stale on any rebuild without a reconfigure, which is
why cr5's binary said 6ce7ad0 while its manifest said 1a097dc. The build
stamp below still disambiguates two binaries built from the same
revision, which a revision alone cannot.
```

### `explo_planner_node.cpp:5137-5140`

**Documents:** `exp_log_->addParamBool("reconnect_enabled", reconnect_enabled_);`

**Cites:** campaign, `cr3`, `cr5`

```
Both names, one resolved value. `reconnect_enabled` is current;
`rendezvous_enabled` is kept verbatim and forever, so the cr3-cr5
analysis and the gate scripts read a new log exactly as they read an
old one, whichever name the harness passed.
```

### `explo_planner_node.cpp:5270-5276`

**Documents:** `exp_log_->addParamNum("goal_rotate_timeout_sec", goal_rotate_timeout_sec_);`

**Cites:** campaign, count, `g6pilot`

```
The thresholds every nav_goal_failed row is tested against. Previously
absent from the echo entirely, which meant the number that decided 30 of
31 failures in the g6pilot campaign (goal_rotate_timeout_sec = 15 s)
appeared NOWHERE in the machine-readable output and was recoverable only
by reading the YAML by hand. nav_budget_sec is per-goal — computed from
goal distance by navBudgetSec — so it cannot be echoed as a constant;
echoing its four inputs makes it reconstructible instead.
```

### `explo_planner_node.cpp:5715-5746`

**Documents:** `rendezvous_schedule_enable_ = dp("rendezvous_schedule_enable", false);`

**Cites:** campaign, count, `mh1`, `ts4`

```
The MIDPOINT floor that used to backstop this is gone (2026-09-16; see the
three removal notes in dispatchReconnect / pursuitFallback). It guaranteed
a destination when no tour cell was worth its detour, but a place with no
agreed time is not a rendezvous, and the floor_won telemetry showed it was
not even symmetric — both ends chose it in only 2 of 6 separated pairs.

THERE IS STILL A FLOOR. Passing none was tried for two days and cost the
ts4 smoke its whole N=4 rung: with empty tours the candidate set was empty
and the solve refused, once, 578 s before the end of a run that then never
held an appointment at all. Since 2026-09-18 `floor_cell` is the centroid
of the team's own vehicle cells — a quantity that, unlike the midpoint,
exists at proposal time and is agreed by construction, because it comes out
of the frozen shared snapshot every robot holds. See the floor block in
deriveRendezvousProposal for the full argument. A refusal therefore still
means something, but it now means the solve declined on cost, not that it
had nothing to look at.

WHY, in one measurement. Campaign mh1 ran the midpoint destination under
the §3.6 value gate: 100 of 110 evaluations declined, every single one on
COST (c_re > c_no), never on knowledge. That is not a gate being cautious,
it is the gate correctly reporting that the destination is worthless — the
midpoint of two poses that were in contact sits BEHIND both robots on
ground they have already covered, so meeting there always costs more than
it returns. Nothing about the gate can fix a bad destination; the
destination had to change.
```

### `explo_planner_node.cpp:6002-6017`

**Documents:** `const double mutual_ttl = team_model_.config().direct_ttl_sec;`

**Cites:** measured

```
THE MUTUAL-CONTACT BUDGET. Not the commit rule's — the commit latches
confirmations and consults no TTL at all, deliberately, because a peer's
choice of pair is a decision it cannot revise rather than a liveness
claim that can age out. What DOES run on a TTL is everything that has to
be true NOW: TeamModel::Peer::direct, and through it
rendezvousTeamMutual(), and through that the proposer's derive gate and
rendezvous_anchor_time_ — the shared origin every agreed interval is
measured from. A TeamWorld rate slower than that TTL means peers read
stale more often than fresh, the team never reads mutually whole, and
the protocol stalls one step earlier than it used to: no pair is ever
derived, so there is nothing to confirm.
```

### `explo_planner_node.cpp:6447-6455`

**Documents:** `auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(`

**Cites:** measured

```
On its own timer rather than the planning tick: a robot that stops
publishing its world while it is busy planning goes silent from its
peers' point of view at exactly the moment it has most to say, and the
TTL cannot tell that apart from a radio outage.
SIM clock, like every other timer in this node. A wall timer would pace
the broadcast in real seconds while direct_ttl_sec is measured in sim
seconds off this->now(), so at RTF 0.3 a healthy peer publishing at
"1 Hz" would arrive every 3.3 sim-seconds and a 5 s TTL would be one
dropped message from declaring it lost.
```

# Runtime logic — planner, manoeuvres and telemetry

### `explo_planner_node.cpp:7150-7167`

**Documents:** `const double held_for = missionElapsed();`

**Cites:** banked, measured

```
Measured from missionElapsed(), which is each robot's OWN baseline —
about a second apart across a fleet in the banked smokes. That is the
right clock even so. The hold is not an appointment and nothing has
to line up on it; what it has to do is guarantee a stretch of
co-located mutual contact, and a second of skew at the far end of a
60 s window does not threaten that. Using a shared wire time here
would import the agreement problem the hold exists to solve.
```

### `explo_planner_node.cpp:7429-7437`

**Documents:** `RCLCPP_INFO(get_logger(),`

**Cites:** banked, campaign, `g6pilot`

```
The outcome is IN the line because it was not, through generation 7, and
the plaintext log therefore could not say how any manoeuvre resolved.
The destination state is not a proxy for it: under mission return every
manoeuvre ends "-> RETURN_HOME" whether it reconnected or gave up, so a
log-based classifier had nothing to key on. Across the 5 banked g6pilot
firings it labelled 4 "open_at_horizon" — never ended — beside a duration
it had just parsed from this very line, and the 5th "arrived_waiting" off
a stray meeting-point line. 0/5 correct against the jsonl, which records
one genuine reconnection among them.
```

### `explo_planner_node.cpp:7494-7581`

**Documents:** `const int live_now = accountedPeerCount(this->now());`

**Cites:** campaign, count, `ts4`

```
This read rendezvousTeamMutual() — every pair in DIRECT contact — while
doReturnSync releases the robot on teamComplete(accountedPeerCount):
every peer *I* have heard first-hand, on the claim table or on TeamWorld.
The two differ by ONE-WAY CONTACT, by MY EDGES vs ALL PAIRS, and by
FINISHED PEERS — but NOT by relayed reachability: accountedPeerCount's
two reachability channels are both first-hand, see the correction at the
release site in heartbeatTick(). Its third channel is `finished`, which
does relay and has no TTL, so a peer that ended its run counts toward
teamComplete forever and can never be mutual; on that peer the barrier
releases and rendezvousTeamMutual() never will.
At N=2 the all-pairs quantifier collapses onto the single peer, so there
the gap is mutuality alone; at N>=3 they part company on both counts, and
the ts4 N=3 cell wrote both verdicts in the same millisecond:
```

### `explo_planner_node.cpp:7830-7854`

**Documents:** `const char* budget_src = "";`

**Cites:** campaign, `g8r1`

```
Cost is one extra ROI walk per PLAN tick spent at a spent budget, NOT
"once per run" as this said through generation 7. finishOrRendezvous's
return value is discarded here, so a DEFERRED decision (rendezvous
confirmation window) leaves the node in PLAN and re-walks on the next
tick until the window resolves. It is bounded by reconnect_confirm_sec
(3.0 s) and stamps nothing twice — recordExplorationComplete is keyed on
step_, which cannot advance during a deferral. Unreachable in the g8r1
configuration regardless: finishOrRendezvous returns at the
mission_return_enabled_ && have_home_ branch, above the deferral gate,
and startReturnHome always returns true. Documented because the walk is
a grid traversal and "once per run" is what a reader would budget for.
Two statements, deliberately. Written as a single nested call the two
arguments are only INDETERMINATELY SEQUENCED, and g++ evaluates the
bare `budget_src` first at both -O0 and -O2 — so the out-parameter the
inner call fills arrives at the outer call as the "" it was initialised
to, noteCoverageDecisionSample's `if (source && *source)` guard rejects
it, and last_coverage_source_ silently keeps the PREVIOUS decision's
source. Which is this fix defeating its own purpose: recording the
source this decision was actually taken on is the whole point.
```

### `explo_planner_node.cpp:8036-8050`

**Documents:** `reconnect_terminal_ = false;`

**Cites:** campaign, measured, `ts1b`

```
An appointment-due departure is a MID-RUN manoeuvre: exploration is not
over, the deadline simply arrived. Say so explicitly, because
reconnect_terminal_ is ambient state whose RESTING value is `true` (it
is declared true and re-armed true at every manoeuvre end), and this
site reaches startReturnTo without passing either of the two places
that set it for a dispatch -- the mid-run clear below, or
finishOrRendezvous's terminal set. Inherited, the barrier that ends
this manoeuvre takes the run-ENDING wait cap (rendezvous_max_wait_sec)
instead of the mid-run one, so a deadline departure can finish the run.
Measured on ts1b: 40 barrier give-ups, every one flagged terminal, none
of which any dispatch had earned.
```

### `explo_planner_node.cpp:8057-8121`

**Documents:** `if ((reconnect_mode_ == ReconnectMode::RENDEZVOUS ||`

**Cites:** count, measured

```
Gen 10 armed it from dispatchReconnect instead, behind the mid-run gate,
and the gate needs reconnect_midrun_silence_sec of team-incomplete before
it fires. Whenever the agreed interval was shorter than that gate the
meeting time had already passed before any robot was permitted to look at
it: measured on a 2-robot cell, anchor 240 s, interval 20 s, armed at
378.6 s and 438.1 s, both robots recording 234.1 s of lateness on an
appointment neither could ever have kept. The place was agreed and the
time was decorative.

What the exclusion DID cost was symmetry, which is the one property this
whole protocol exists to provide. Arming only from dispatchReconnect put
hybrid's appointment behind the mid-run gate, and that gate is per-robot
and genuinely asymmetric: the link-gate veto is computed from each robot's
OWN unshared backlog, and it refuses often (102 of 115 refusals on mt2
were the cost inequality alone). So one hybrid robot's gate says go and
arms, the other's says stay and never arms — and the first drives to the
agreed cell and waits out the full 240 s cap for a partner that was never
coming, logging a no-show against an appointment only one end ever held.
A meeting one participant does not know about is not a meeting, and
"hybrid keeps a rendezvous" has to mean the same thing on both robots or
the arm is not testing what its name says.
```

### `explo_planner_node.cpp:8126-8267`

**Documents:** `const int live_for_arm = accountedPeerCount(this->now());`

**Cites:** measured

```
WHAT IT COSTS, measured rather than asserted. Complementarity makes the
team-wide predicate equivalent to an all-pairs complete graph, and in the
generation-22 smoke (same radio regime) the full mesh held 29.0% of the
run at N=3 rendezvous, 41.1% at N=3 hybrid and 14.7% at N=4 under the 5 s
liveness TTL. So at N>=3 these arms now stand armed for most of the run.
That is NOT a permanent manoeuvre: per P5 above a standing appointment is
a CLOCK, not a decision — robots keep exploring until the departure
deadline — so what it produces is periodic rendezvous at the agreed
interval, which is the mechanism as described. It IS a large behavioural
change and generation 23 is not comparable with 22 on these arms.
```

### `explo_planner_node.cpp:8270-8283`

**Documents:** `armAppointment(teamComplete(live_for_arm, rendezvous_expected_peers_)`

**Cites:** banked

```
TWO REASONS, ONE PREDICATE. The arm is unconditional on !teamSettled;
the string only records WHICH half of it fired, so a contagion arm can
be counted without a new column or a schema bump. "separation" is this
robot's own read failing — the pre-generation-23 behaviour, unchanged,
so banked parsers that expect it still find it. "peer-separation" means
this robot can hear everyone and armed SOLELY because a peer announced
a break: it is the C of the A-B-C bridge, and the count of these is how
much the contagion actually did. Nothing in the tree matches on this
string, which is what makes a second value safe here.

WHERE IT LANDS: the plaintext "Rendezvous schedule [%s]" line in
armAppointment, NOT the jsonl — RendezvousAgreedEvent has no reason
field and adding one would bump a schema that several banked parsers
pin. grep the cell's planner logs for "[peer-separation]".
```

### `explo_planner_node.cpp:8351-8407`

**Documents:** `bool   link_veto     = false;   // stand down this tick`

**Cites:** banked, campaign, count, measured, `g8r1`

```
missing_for is the TEAM-PRESENCE clock, not the peer's record age:
team_last_complete_time_ is stamped on the 1 Hz heartbeat while
accountedPeerCount() reads the team complete, and a peer stays
accounted until BOTH its claim (coord_claim_ttl_sec, 5 s) and its
TeamWorld `direct` flag (TeamModel::direct_ttl_sec, 5 s) age out. So
missing_for ~= peer_record_age_sec - TTL. Measured on g8r1's 5 banked
mid-run dispatches the difference is 4.98-4.99 s (4.99, 4.99, 4.99,
4.99, 4.98) -- the TTL, from below, to two decimals. A previous
revision of this comment quoted 4.98-5.01; there is no 5.01 in the
five, and an upper end ABOVE the TTL is not a value this quantity can
take, so the typo was also self-refuting.
An earlier draft widened this to 3.24-5.51 "because the only
banked copy is a %.0f-rounded plaintext number"; that was wrong.
team_incomplete_sec is a real 2-dp column and generation-8 cells DO
carry it, so no rounding allowance is needed.

TWO OFFSETS, NOT ONE, AND THE EARLIER "T + 5" CONFLATED THEM.
The TTL offset above is the gap between the two CLOCKS. Separately,
the trigger is evaluated on PLAN ticks, so the presence clock
overshoots T before anyone looks: measured 9.02-13.82 s past 240.
The record age at dispatch is therefore T + overshoot + TTL, and the
5 banked dispatches sat at 254.01-258.80 s against a nominal 240,
i.e. T + 14 to T + 18.8 -- not T + 5. Only the TTL half is
threshold-independent; the overshoot is set by tick cadence, which is
why this is quoted as a measured range and not as arithmetic. At
generation 9's T = 90 the same decomposition predicts a dispatch
record age around 104-109 s.
team_incomplete_sec is logged as its own column so the fired
inequality is recoverable offline at full precision.

Timing on link-down duration instead was tried and rejected against
the banked data. The two quantities are not variations of each
other: record age accumulates ACROSS outages (the beacon is
conditional, so silence spans up-periods), while continuous outage
resets at every flicker. On tl1's 30 hybrid cells only 1 of 286
outages ever reached the campaign's 240 s gate, so timing on it
would have dropped 17 of 19 mid-run fires — switching mid-run
pursuit off rather than correcting it, under a threshold that was
never tuned for that quantity. The veto drops 4 of 19: exactly the
fires that went out to a peer already on the radio. Those 4 do not
all arrive here — 3 came through this trigger and 1 through the
exhausted-chase escalation in pursuitFallback, which is why the veto
is applied at both sites and why gating only this one left a leak.
```

### `explo_planner_node.cpp:8414-8435`

**Documents:** `if (link_connected_ ||`

**Cites:** campaign, `g8r1`

```
Debounced on reconnect_link_down_confirm_sec, which is the RADIO
debounce and nothing else. It used to borrow reconnect_confirm_sec
(the team-presence release confirm, 3.0): two unrelated questions
on one constant. Also makes link_down_sec on any dispatch
unambiguous: >= the confirm when the gate decided, -1 when the gate
was not in play at all (every g8r1 dispatch reads -1, which is how
we know that campaign ran with no veto).
```

### `explo_planner_node.cpp:8498-8514`

**Documents:** `gate_evaluated = true;`

**Cites:** count

```
BUILT HERE, WRITTEN AFTER THE DISPATCH (2026-09-16). `dispatched`
used to be assigned the gate VERDICT and the row written before
the dispatch was attempted, so the two disagreed on exactly the
population that matters: a gate that said go, an attempt duly
spent, and dispatchReconnect then returning false because a
standing appointment had already claimed this outage's one
chase. Those rows said dispatched=true with no manoeuvre
anywhere in the run, which is the same shape as the checks that
stopped checking — a column reporting its failure case as its
success case. It now reports what actually happened. The flip is
one-directional (only true -> false, only on that population),
so a row that says dispatched on an older generation still means
what it meant; it is the gen-15 rows that gained a distinction.
```

### `explo_planner_node.cpp:8526-8530`

**Documents:** `ev.attempts_used       = midrun_attempts_;`

**Cites:** banked

```
Deliberately still PRE-increment: "attempts already used when
this evaluation ran". The increment below happens after this
row's decision, and moving the read past it would silently
shift every value in the column by one against every banked
generation.
```

### `explo_planner_node.cpp:8764-8810`

**Documents:** `constexpr float kCostEpsilon = 0.1f;`

**Cites:** measured, stat

```
WHY THE KNOB EXISTS. Measured over 703 logged decisions on flatforest_dense
(campaign p14, off arm, to the 0.60 unknown rung): across the candidate set
at a single decision, path_cost spans roughly 5.8x while info_gain spans
only ~0.35 sd/mean. Cost enters linearly and varies far more, so argmax(U)
collapses to argmin(cost) — the planner chose goals at a median 7.8 m when
the mean candidate was 45.2 m away, i.e. it ran as nearest-frontier. The
same collapse is described from the other direction in shared_params.yaml
at candidate_min_goal_dist_m ("6% spread against a ninefold spread in
path_cost"). Solving for the γ at which a mean+2sd-information candidate at
the field's mean distance overtakes the one actually chosen gives a median
of 0.25 (p10 0.15, p90 0.42).

WHAT IT DOES NOT FIX, stated so γ is not mistaken for a repair. Against the
map actually gained afterwards, info_gain has Spearman ρ ≈ +0.18 — real
(the null, raw distance, is ≈ 0) but weak, and its ~1.7x span cannot
separate outcomes that range over 600x. Lowering γ stops a nearly-flat
information term from being overruled by cost; it does not make that term
discriminate. The repair is the information model, not this exponent.
```

### `explo_planner_node.cpp:8951-8976`

**Documents:** `const int self_idx = fleet_.self_id;`

**Cites:** count

```
This band used to be 0, on the reading that "unrestricted" meant
"competing on utility with the focus neighbourhood". Rank 0 is not an
exempt band, it is the WINNING one: the comparator is lexicographic on
rank before utility and the walk takes the first admissible candidate,
so a ring point beat every out-of-focus frontier no matter how much
more that frontier would have revealed. The ring is generated around
the robot and is almost always admissible, so in the p4 smoke the pick
was rank 0 on 29 of 29 allocation rows while `reordered` was true on
27 — the tour was solved, logged, and never actually steered anything.
The demotion below made it self-sustaining: it reads `picked_rank == 0`
as "came from the focus", so a ring pick reset the staleness counter
that exists to write off a focus cell nothing can reach, and the focus
wandered the grid a cell per tick.
```

### `explo_planner_node.cpp:9339-9369`

**Documents:** `const int focus = alloc_ev.focus_cell;`

**Cites:** measured

```
Staleness (§3.4). A focus cell that keeps yielding no admissible
candidate is re-measured from the map and, if it still claims to be
worth exploring, demoted to COVERED — mTARE's not-connected -> COVERED
demotion, and the only thing that stops a phantom EXPLORING cell from
being re-assigned forever and from dragging the rendezvous minimax
toward ground nobody can clear.
```

### `explo_planner_node.cpp:9510-9516`

**Documents:** `pending_sep_eligible_peers_ = static_cast<int>(sep_anchors.size());`

**Cites:** measured

```
The first two are measured whether or not the term is on. In an untreated
arm they are the counterfactual: how close to its teammate did the
unmodified planner send this robot, and did it even have a teammate to be
close to. Without them a null result cannot distinguish "dispersion does
not help" from "the robots were never near each other anyway".
```

### `explo_planner_node.cpp:9524-9556`

**Documents:** `pending_sep_reordered_ = -1;`

**Cites:** measured

```
Not asked on three kinds of tick: the term is off (with no discount the
two orders are the same by definition, so 0 would be true but vacuous, and
-1 keeps it from being pooled with a measured 0); the walk selected
nothing; or the pick came from the amnesty fallback, whose ordering is by
failure age rather than by utility and which the term therefore cannot
reach.
```

### `explo_planner_node.cpp:9682-9687`

**Documents:** `*source = "scovox";`

**Cites:** measured

```
scovox source: 2.5D column coverage of the ROI footprint, measured on the
fused 3D map already ingested (ROI + z-band clipped) into map_cache_ by
loadLatestMap() this tick. NB in flat mode the ingest band is the absolute
[roi_min_z, roi_max_z]: on terrain outside that band the cache stays empty
and this reads 1.0 (never done) — set a per-area z band or
terrain_relative_z when the ground leaves the default band.
```

### `explo_planner_node.cpp:9805-9822`

**Documents:** `const bool yaw_required =`

**Cites:** campaign, `ts1b`

```
The yaw term's whole justification was "so the sensor actually observes the
region the planner scored". That is a statement about the SENSOR MODEL, and
it is now enforced as one: an omnidirectional model scores a full circle
from the goal position, so every heading observes what was scored and
holding the robot still until it reaches a nominal yaw buys nothing. It is
not free, either — the rotate deadline below fails the goal and failGoal()
blacklists the position the robot is standing on, which is how ts1b
produced 984 `budget-rotate` failures on a sensor that does not have a
front.
```

### `explo_planner_node.cpp:9885-9892`

**Documents:** `auto now = this->now();`

**Cites:** measured

```
At XY, waiting for the controller to finish rotating. This gets its OWN
deadline, armed on arrival. Reusing nav_budget_sec_ was wrong twice
over: that budget covers the DRIVE and is measured from NAVIGATE entry,
so a robot that arrived at 15.9 s against a 16 s budget was failed on
the very next tick with the goal already underfoot — and failGoal()
blacklists current_goal_.position, so the robot then rejected every
candidate within failed_goal_radius_m of where it was standing for the
whole failed_goal_ttl_sec.
```

### `explo_planner_node.cpp:9961-9971`

**Documents:** `RCLCPP_WARN(get_logger(),`

**Cites:** measured

```
The comparison that fired is logged with the failure, not left at DEBUG
where the campaign never captures it, and it is printed as the TESTED pair
rather than as (elapsed, nav_budget). The old line printed the latter for
all three reasons, which made every budget-rotate row read "failed at 43-58 s
of a 180 s budget" — a claim about ground the robot could not cross — when
the actual test was a 15 s rotation timeout. A "budget" failure at 31 s is a
different animal from one at 179 s (estimator wrong about a short hop vs
genuinely impassable ground), and that distinction only survives if the
threshold printed is the one that was consulted. `pose_stale` marks an
attempt whose progress metric was measured against a pose that stopped
updating — see the pose-health event.
```

### `explo_planner_node.cpp:10347-10353`

**Documents:** `std::vector<double> deltas;`

**Cites:** measured, stat

```
Winsorised slope: per-interval deltas, each capped at kRateWinsorK x the
window's median delta, summed over the window's total elapsed time. The
cap is what makes this a PRIVATE gathering rate rather than a total one:
a reconnection merges the peer's map into this same cumulative count, a
step of up to 243k voxels in ONE sample (measured, p14), which a plain
two-point slope reports as growth this robot never sensed. Deltas are
floored at 0 for the mirror artifact (a map reload shrinking the count).
```

### `explo_planner_node.cpp:10439-10442`

**Documents:** `if (est_unshared_out != nullptr) {`

**Cites:** measured

```
Diagnostic backlog estimate: my delta is EXACT (own cached count), the
peer's is dead-reckoned at its beaconed rate. Logged on the dispatch
event so the offline analysis can score the estimator against the
transfer actually measured at the merge.
```

### `explo_planner_node.cpp:10447-10458`

**Documents:** `const double rate_sum =`

**Cites:** measured

```
SUM, not mean: once each side's rate is a PRIVATE gathering rate (the
winsorised estimator in noteMapSize — a total-count rate would already
include the peer's contribution and summing it would double-count), the
backlog is the union of two disjoint gatherings and its growth is their
sum. Checked, not assumed: summed = 1.12x p14's measured pair divergence,
where the mean would read 0.56x and fire roughly twice too late.
```

### `explo_planner_node.cpp:10666-10715`

**Documents:** `return appointment_manoeuvre_`

**Cites:** banked, measured

```
THE REACHABLE DOOR (2026-09-19, generation 27). teamSettled is full mesh
over this robot's own edges plus the contagion, and at the meeting point
that is a GEOMETRY test, not a patience test: under 70 dB trunks one tree
on one chord keeps one pair dark at 8 m forever. The gen-26 N=3 smoke
measured exactly that — pair 0-2 up 1.3% of the window with both other
pairs at 100%, all three robots gathered at the agreed cell and parked in
RETURN_SYNC from ~350 s to the 660 s cap, zero rendezvous_outcome rows,
the cell banked CLEAN. Waiting cannot close such a pair, so the barrier
also releases when every expected peer is REACHABLE — in TeamModel's
comms closure, or finished — i.e. "until all robots are connected" read
as one connected component rather than all pairs. What the dark pair
could not exchange it still has not (there is no map relay); the meeting
delivered every exchange its radio physically allowed, and parking longer
was buying nothing.
```

### `explo_planner_node.cpp:10780-10783`

**Documents:** `const double t_derive = missionElapsed();`

**Cites:** measured

```
The origin the published instant is measured from. Refused rather than
defaulted: a negative mission clock means this node has not ticked live
yet, and an appointment stamped in a frame that does not exist yet is one
the peers would evaluate against a different zero.
```

### `explo_planner_node.cpp:10867-10891`

**Documents:** `int floor_cell = -1;`

**Cites:** campaign, `ts4`

```
The floor used to be the midpoint of the LAST-CONTACT pose pair, and that
quantity does not exist at proposal time — the proposal is derived while
everyone is still connected, so there is no last contact to take a midpoint
of. Passing -1 instead was the obvious reading, and it cost the ts4 smoke an
entire arm: the only refusal the N=4 rung logged was "the tours are empty
and the last-contact midpoint is outside the ROI". The team was mutually
whole for about two seconds, the allocator had not produced a tour yet, the
candidate set was therefore EMPTY, and the run continued for another 578 s
with no appointment at all.
```

### `explo_planner_node.cpp:11010-11037`

**Documents:** `const double t = missionElapsed();`

**Cites:** banked, campaign, measured, `ts4`

```
AND THAT WINDOW IS NOT RADIO-LIMITED, which is the part worth acting on.
Measured on the banked ts4 N=4 rendezvous cell's link_states.csv, all six
pairs are simultaneously connected from t_sim 5.0 s to 46.4 s — a 41.4 s
full mesh — and then never again for the remaining ~600 s of the run. The
node's own rendezvousTeamMutual() recognised about two seconds of those
41.4. The two numbers are different quantities and do not contradict each
other (one is the emulator's link truth, the other is received-intent
freshness sampled on the heartbeat), but the GAP between them is the whole
diagnostic: at N=4 the derive is starved by how the node reads contact, not
by how long the team has it. Relaxing the gate to spanning connectivity
does NOT recover it — the same file gives only 10.2% spanning against 6.4%
full mesh, and after t=46.4 s just 4% of the run — so the lever is intent
freshness inside the one early window, not a weaker topology test.
```

### `explo_planner_node.cpp:11062-11233`

**Documents:** `if (proposer && team_mutual && have_rendezvous_snapshot_ &&`

**Cites:** banked, campaign, count, measured, `ts4`

```
It used to re-derive every rendezvous_proposal_period_sec_, and every
re-derive opened a commit race: the proposer publishes generation N+1, the
followers adopt and commit it one TeamWorld period later, and the proposer
cannot commit until it sees those echoes a further period after that. For
the round trip in between, the fleet genuinely holds two different pairs.
That was harmless while arming waited for the silence gate — the race was
long over by the time anyone looked — but arming now happens at the
separation itself, so a separation landing inside that window arms a
SPLIT fleet. Measured on a 2-robot cell: three outages, two agreed exactly
(cells 57/57 and 27/27, anchors 0.5 s apart), and the third had one robot
on cell 56 at t+28 s and the other on cell 57 at t+88 s. It waited at the
agreed place; its partner waited at a different agreed place; the run
recorded a no-show against an appointment both had kept faithfully.

THE DEFECT IT WAS AIMED AT IS REAL. The bound above — once, and only while
mutually whole — is not enough, because `team_mutual` is a claim about the
last few seconds rather than about the next few. In ts4 smoke20's N=3
hybrid cell the proposer authored the upgrade 0.6 s after its last peer had
gone silent: mutual was still reading true off a tolerance window that had
not expired yet. The peer never received the upgrade, armed on the
placeholder it had committed 26 s earlier, and the fleet kept two exact
appointments in two different cells for the remaining 400 s of the run.

WHY THE HOLD CANNOT BE THE CURE. The upgrade fires when the argmin has more
than one admissible cell to choose between, and the candidate pool is the
union of GlobalAllocator's tours, which are built only over cells in
EXPLORING/EXPLORING_BY_OTHERS. A robot parked in WAIT_FOR_MAP completes
zero steps and therefore grows no such cells. Measured over every banked
cell that carries agreed_provisional (5 campaigns, 10 cells, 28 robot-runs,
58 rendezvous_agreed rows):

A hold therefore does not DELAY the upgrade, it DELETES it: the branch
below is the only site that ever clears rendezvous_held_provisional_, so a
window that closes before the first non-provisional derive leaves the pair
provisional for the rest of the run. Both scheduled arms would then measure
"meet at the team's initial centroid" while passing every unanimity check,
because a fleet that unanimously agrees a placeholder is still unanimous.
Two banked upgrades that the confinement would have rejected outright:
smoke18_n2 hybrid at t=311.1 s and smoke20_n3 rendezvous at t=353.6 s, both
more than 250 s after the provisional they replaced.

WHY NOT REQUIRE AN ECHO ROUND FOR THE UPGRADE, as the provisional pair
requires. Also tried on 2026-09-17, also withdrawn: at N>=3 each FOLLOWER
then waits on the OTHER FOLLOWERS' echoes, over follower-to-follower links
this radio regime need not provide, and the measured result was a proposer
that committed while its followers did not — the same split, with the
proposer on the far side of it.
```

### `explo_planner_node.cpp:11346-11363`

**Documents:** `RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,`

**Cites:** campaign, `ts4`

```
THIS IS NOW AN ABNORMAL OUTCOME. It used to be the normal early-run
answer — no tours to insert into, no floor, nothing admissible — and
the centroid fallback removed exactly that case: a refusal here now
means the snapshot has no locatable robot in it or the world is not
configured, not that the run is young.
THE COUNTS TRAVEL WITH THE REFUSAL. Without them this line cannot
distinguish "no tours yet" from "every tour cell was filtered out",
and those want opposite remedies. It is also, on the evidence of the
ts4 N=4 rung, frequently the ONLY record that an entire arm ran
without a treatment: the derive is gated on team_mutual, which at
N=4 was true for about two seconds, so this fired once and the run
continued for another 578 s with no appointment and every gate
green. A single throttled WARN is not an adequate witness for that,
which is why the campaign gate now fails the cell outright — but the
WARN is what says WHY, so it has to carry the arithmetic.
```

### `explo_planner_node.cpp:11387-11440`

**Documents:** `const RendezvousProposal& p = rendezvous_peer_[kRendezvousProposerId];`

**Cites:** campaign, `ts4`

```
THE ONE UPGRADE, MIRRORED (2026-09-17). This branch has to take the same
exception the proposer's derive gate takes, and it has to take it from the
SAME evidence, which is why `provisional` is on the wire and not just in
the proposer's head. Without it this code was not merely incomplete — the
upgrade could not succeed in ANY interleaving. Either every follower
ERRORed and kept the placeholder while the proposer published an orphan
(the upgrade a structural no-op, plus a permanent ERROR), or, if the
proposer had not yet seen every echo when it re-derived, the proposer
committed nothing at all and refused every arming for the rest of the run
while its followers drove the placeholder without it. The second shape is
the ts4 N=4 arm death reproduced by the code written to fix it.
```

### `explo_planner_node.cpp:11563-11603`

**Documents:** `if (!rendezvous_held_.valid()) return;`

**Cites:** measured

```
MUTUAL CONTACT IS NOT TESTED HERE EITHER, and this is the substantive
half. The old rule took it because a one-way link satisfies freshness, and
committing under one would mean "concluding the team holds a pair my own
echo may never have reached". The latch closes that hole directly instead
of approximating it: a latch is only ever written from a message RECEIVED
from that peer carrying that exact pair, so it is first-hand evidence that
the peer holds it. This robot never infers a peer's state from its own
transmissions, so there is nothing for one-way contact to break. The
symmetric condition still guards the one quantity that genuinely needs
both directions — rendezvous_anchor_time_, which is the shared origin every
t_meet is measured from, and which is still stamped only under
rendezvousTeamMutual().
```

### `explo_planner_node.cpp:11616-11730`

**Documents:** `if (on_pair != fleet_.size() - 1) return;`

**Cites:** banked, campaign, measured, `ts4`

```
Measured, in ts4 smoke20's N=3 hybrid cell: bestla lost its last peer at
t+65.1 s, the proposer authored R at t+65.7 s, and bestla armed on the
placeholder while atlas and husky armed on R. Two exact appointments, two
cells, 400 s of the run spent waiting in the wrong places.

THE RULE STAYS ANYWAY, because reverting it restores a different split —
the N=3 follower-to-follower deadlock measured directly above, which is not
hypothetical either. Neither rule is correct on its own, and no rule is:
committing an upgrade atomically across a partitionable fleet is the Two
Generals problem, so every candidate protocol just chooses which robot gets
left behind. Generation 22 tried to change WHEN this may run — confining
the upgrade to the pre-mission hold, while the fleet is co-located — and
that was withdrawn on measurement the same day: the upgrade's input is
completed exploration steps, a held robot completes none, and the
confinement deleted the upgrade instead of scheduling it. See the derive
gate in maintainRendezvousProposal for the numbers.

THE ECHO ROUND IS BACK, FOR EVERY COMMIT AND EVERY N (2026-09-18). The two
paragraphs above are kept because both failures they describe are real and
measured, and someone will otherwise re-derive one of them. What changed is
not the argument, it is the thing being argued about: BOTH of those splits
are splits between a robot on R and a robot on P, and both were harmful
only because P was a placeholder rather than a usable agreement.

WHAT IT COSTS is the case the note at the derive gate warned about: an
upgrade that never lands leaves both scheduled arms measuring "meet at the
team's initial centroid". That is no longer a silent degradation — the
provisional flag is on the AGREED log line and in the banked row, so a cell
that never upgraded is visible as such.
```

### `explo_planner_node.cpp:11752-11784`

**Documents:** `RCLCPP_INFO(get_logger(),`

**Cites:** banked

```
THE PROVISIONAL FLAG IS ON THE LINE because a cell where the whole team
committed the centroid placeholder and a cell where it committed a
tour-informed meeting point are measuring different mechanisms, and
without this they are indistinguishable in the banked logs. Both are
valid runs; only one of them exercised the scheduler's argmin. A team
that commits provisional and upgrades later prints this line twice.

It was dropped the same morning as false, and it was: armAppointment had
been changed to overwrite t_meet with a private countdown, so the line
described a schedule the code did not keep. The countdown is gone and the
schedule is the thing armAppointment attends, so the line is a statement
about behaviour again — and it is the ONLY place the agreed meeting time
reaches the banked text logs, which is what an offline reader needs to
check that two robots armed the same MEETING rather than the same cell.

THE ECHO COUNT IS REPORTED SEPARATELY from the fleet size, and now they
agree by construction: the early return above requires on_pair ==
fleet-1 for every commit, so "all N robots" is an OBSERVATION and not, as
it was for one day, an inference from the protocol. Both are printed
anyway. If they ever differ in a banked log, the commit rule regressed,
and a reader who only has the text log should be able to see that.
```

### `explo_planner_node.cpp:11804-11821`

**Documents:** `rendezvous_agreed_peers_ = on_pair;`

**Cites:** banked, count

```
It was a genuine measurement for one day, while a final triple committed on
first-hand evidence and a row could legitimately carry on_pair < fleet-1.
Anything analysing the banked corpus has to read it that way PER GENERATION
and not pool: the same column means "how many peers had echoed" on gen 22
and "fleet-1, always" on gen 23.

Kept rather than replaced by a constant because it is the one place a
regression in the commit rule would show up in the data rather than only in
the code — a gen-23 row carrying on_pair < fleet-1 means this return was
bypassed.
```

### `explo_planner_node.cpp:11825-11860`

**Documents:** `bool ExploPlannerNode::armAppointment(const char* reason) {`

**Cites:** campaign, `ts4`

```
EVERY INPUT HERE IS AGREED OR SHARED. `t_meet_ms` and `interval_ms` are the
integers the whole team committed, byte-identical on every robot and part of
RendezvousProposal::operator== so a fleet cannot hold two of them and report
agreement. `not_before_sec` is this robot's mission clock, whose baselines
are ~1.2 s apart across the ts4 cells. So two robots disagree about k only
when their `not_before` values straddle a multiple of the interval, and when
they do the disagreement is bounded by ONE interval and resolves at the
barrier rather than sending anyone to a different place.
```

### `explo_planner_node.cpp:11956-11979`

**Documents:** `}`

**Cites:** campaign, count, `ts4`

```
The agreed time is now a RECURRING schedule and this arming attends the
first occurrence of it not already past (nextAgreedOccurrence, below),
so the meeting this robot arms cannot be behind it. There is no lateness
left here to refuse on — and, unlike the countdown that briefly stood in
its place, that is true without giving up the agreed instant: the roll
is over the committed integers, and the only per-robot input is the
robot's own `t_now`, which generation 25 made the BARE floor after the
gen-24 `t_now + notice` floor forked the ts4 N=3 cell (the block at the
assignment prices that out).
```

### `explo_planner_node.cpp:11983-12082`

**Documents:** `appointment_             = RendezvousPlan{};`

**Cites:** campaign, count, `ts4`

```
  `anchor*1000 + interval` — anchored on "the instant MY view of the team
  stopped being mutually whole". One event at N=2 (the ts4 N=2 cell put
  the two anchors 0.84 s apart), a per-robot predicate over a graph that
  comes apart edge by edge at N>=3 (13.18 / 34.18 / 33.98 s). The origin
  was private. It is not private here: t_meet is authored once by the
  proposer and adopted verbatim, and it is inside operator==, so a fleet
  cannot hold two of them and still report agreement.

THE FLOOR IS BARE t_now SINCE GENERATION 25, AND THE 100 s NOTICE THAT
USED TO BE ADDED HERE IS WHAT FORKED THE ts4 N=3 CELL. With
`t_now + 100`, the tolerance for arming spread was the gap from the
last floor to the agreed instant MINUS the notice: the cell's schedule
led by 149 s, the notice ate 100 of it, and the 33 s arming spread
(an N>=3 graph dies edge by edge; anchors 72.90 / 72.84 / 90.58 s)
crossed the rest. Two robots floored at ~178/179 s and kept the agreed
190.148 s; the third floored at ~211 s and rolled to 346.716 — base plus
exactly one interval — so its peers stood at the cell 351 and 432 s and
the cell censored with the barrier open. With this floor the same
armings tolerate the WHOLE 149 s gap; only a robot that arms after the
instant has genuinely passed, or that cannot reach it within its
lateness budget, rolls forward — the two cases where rolling is the
truth. The per-robot terms are `t_now` itself, the mission-clock
baseline (`mission_t0_sec_`, latched on each node's first live tick),
~1.2 s across these cells — bounded by node start, not by the radio —
and the arrival shortfall, which is EXACTLY ZERO for every robot that
can make the occurrence and so cannot separate robots that can both
attend. A late-run separation can still straddle an
occurrence boundary by bad phase — no rule computed from private
observations can prevent that — but the fork is no longer manufactured
by subtracting a constant from the margin, and the barrier prices a
residual fork at one interval of waiting rather than a censored run.

THE BUDGET IS SUBTRACTED FROM THE DRIVE, NOT ADDED TO THE FLOOR, and the
clamp at zero is the whole safety argument. Generation 19-24 set the
floor to `now + rendezvous_depart_delay_sec`, a lead time every robot
added unconditionally, so robots that could all comfortably make the same
rung still split across two of them — the ts4 N=3 cell, two robots on one
occurrence and the third a whole interval past it. Here the floor is bare
t_now for every robot whose marked-up drive fits inside the margin plus
the budget, which is the common case and cannot fork. It rises above
t_now only for a robot that genuinely cannot arrive in time, and then by
exactly its shortfall. Clamped at zero it can only ever move the rung
LATER, so the "deadline already passed" impossibility below still holds.

A FORK HERE IS PRICED, NOT PREVENTED. A robot that rolls is one interval
out of step with peers that did not, and the barrier's unbounded wait
pays for that in standing time. It is the better trade: the alternative
is that robot arriving arbitrarily late — 563 s on the stopped ts4 cells
— with the team standing for that instead, and the rolled robot spends
the interval exploring rather than driving.
```

### `explo_planner_node.cpp:12360-12389`

**Documents:** `const long long lead_ms = appointmentLeadMs(appointment_.cell);`

**Cites:** campaign, `ts4`

```
t_meet is a MEETING instant, so the thing that has to land on it is the
arrival. Between 2026-09-18 and this change it was a DEPARTURE instant —
a bare `now >= t_meet` — and the difference is not cosmetic: under that
rule every robot was late by its own drive, the barrier absorbed the
stagger by making whoever arrived first stand and wait, and across the
stopped ts4 cells that was the whole of the observed waiting. Kalhan,
2026-09-19: robots should only agree to meetings they can come to within a
bounded delay.
```

### `explo_planner_node.cpp:12570-12595`

**Documents:** `if (may_defer) return false;`

**Cites:** count

```
WHY THE MIDPOINT DRIVE IS GONE (2026-09-16). This branch used to fall
through to startReturnTo(meetingPoint(self_pose, peer_pose)) — park at
the midpoint of the two poses at last contact. It was removed from the
RENDEZVOUS arm the same day for two reasons that apply here word for
word: it is a place with NO TIME (nothing tells the peer when to be
there or how long to wait), and it is not actually symmetric (the
floor_won telemetry says both ends picked the midpoint in only 2 of 6
separated pairs). Leaving it in hybrid alone would have been worse than
leaving it in both, because it makes hybrid a THIRD behaviour rather
than the composition of the other two: any hybrid-vs-pursuit or
hybrid-vs-rendezvous contrast would then be confounded by a manoeuvre
neither of those arms can perform, and the factorial reading of the
four-arm design — which is the entire point of running four — would not
hold. The previous justification ("its own mechanism's fallback ladder")
described pursuit's ladder, and pursuit's ladder is the two lines below.
```

### `explo_planner_node.cpp:12613-12617`

**Documents:** `if (pursuitExploreFallback(reason)) return true;`

**Cites:** campaign, `p7modes`

```
Pure pursuit has no agreed fallback point by design (that is the A/B
against hybrid). A chase that never started therefore goes back to
exploring while its fallback budget lasts, and only parks here once
that is spent — parking early is the mutual-hold fixed point that cost
a p7modes mission (see pursuit_explore_fallback_).
```

### `explo_planner_node.cpp:12622-12653`

**Documents:** `if (!may_defer) {`

**Cites:** count, stat

```
  * It has a place and NO TIME. Nothing tells the peer when to be there or
    for how long to wait, which is exactly the property this arm exists to
    test. Pre-P5 that showed up as a median wait of 240 s — the cap, to the
    second — and 16 no-shows in 21 n3 appointments.
  * It is not actually symmetric. The `floor_won` telemetry says so: in
    only 2 of 6 separated pairs did BOTH ends pick the midpoint floor, so
    the shared-by-construction argument was already failing in the data.
```

### `explo_planner_node.cpp:12858-12895`

**Documents:** `void ExploPlannerNode::doReturnNav() {`

**Cites:** campaign, `ts4`

```
— because the settle hold that serves that sentence lives in RETURN_SYNC,
which an en-route RELEASE skips entirely. In a three-robot outage the first
robot to see the team complete would leave immediately while the others were
still driving, so the one hold whose whole purpose is to let the merged map
propagate was paid by nobody. The generation-25 answer is neither of the
gen-24 options (release here, or drive to the ring no matter what): when the
team has SETTLED for the shared confirm window, the walker stops driving and
enters RETURN_SYNC from where it stands — the settle hold is still paid,
the release still happens at the one site that owns it, and the last metres
to the cell are not driven because the thing they were for has already
happened. Gen 24 held that finishing the drive was "the literal reading of
an exact place"; the ts4 24b N=2 hybrid cell priced that literalism: a robot
2.4 m outside a 1.5 m ring, crawling at 0.06 m/s against meeting-point
clutter for 350 s with the link up 90.5% and the maps already merged, its
appointment_inbound bit holding two peers' barriers open to the censor.
```

### `explo_planner_node.cpp:13038-13068`

**Documents:** `bool ExploPlannerNode::deferAppointmentLeg(const char* what_failed, float dist) {`

**Cites:** campaign, seed, `ts4`

```
The watchdogs above end a leg that is not getting anywhere. For an anchor
return, ending it at the barrier is right: the destination was this robot's
own last-connected pose, nobody else was going there, and standing still is
what the manoeuvre wanted. For an APPOINTMENT it is a false claim with
teeth, because the barrier is a presence count and the wait on it is
unbounded by design (rendezvous_appointment_wait_sec = 0.0). A robot that
joins it from outside the cell contributes a presence nobody can observe —
it is not where the others are — and then waits forever for a presence that
is, in turn, waiting for it. Two stationary robots cannot heal a link, so
the pair is wedged until the harness kills the run. That is the ts4 gen-30
seed8 cell in one sentence.
```

### `explo_planner_node.cpp:13126-13171`

**Documents:** `if (releaseConfirmed(manoeuvreReleaseEligible(active))) {`

**Cites:** campaign, measured, `ts4`

```
WHY THE SECOND TERM IS THERE, because the first one alone read as though it
were enough. This block used to argue that "at the meeting point the robots
are within a couple of metres of each other, so every pair should be up by
construction — that is what meeting at one place is FOR". True of robots AT
the meeting point, and the release never required them to be there: it is a
comms test, the radio reaches tens of metres, and the drive it is paired
with only ends on arrival. So the first robot to arrive released as soon as
its partner came into range and left with most of that partner's walk still
to go, and the partner — which may not release en route, the cell being the
agreed thing — finished the walk to an empty cell and waited there alone.
The gen-23 smoke measured 36 s of settle against 272 s of standing.

THE HANG THIS ADMITTED WAS THEN MEASURED (gen-26 N=3 smoke: three robots
gathered at the agreed cell, one trunk on one 8 m chord, all parked in
RETURN_SYNC from ~350 s to the 660 s cap, zero outcome rows) and
generation 27 narrows it. The appointment wait cap is
rendezvous_appointment_wait_sec, which the campaign leaves at 0 =
UNBOUNDED, and through generation 26 "gathers but cannot close every
pair" waited here until the run's duration cap; the reachable door now
releases that shape after the settle. What still waits unbounded is a
team missing a peer from the CLOSURE itself — genuinely absent, still
driving in, or dark to every robot present — which is the vigil the
unbounded cap is FOR. The
finished-peer exemption in peerReportsTeamBreak() removes the one case that
could hold it open indefinitely. The claim that stood here — that the
inbound term ends at the nav budget whether or not the cell was reachable —
was FALSIFIED by the ts4 24b N=2 hybrid cell: the proximity-hold release
refunds held time and restarts the progress window (see doProximityHold),
so a walker crawling against meeting-point clutter held the bit, and this
barrier, for 350 s to the censor with the team connected the whole time.
Generation 25 closes that path at the source: a walker whose team has
settled for the confirm window converts to RETURN_SYNC (see doReturnNav),
which clears appointment_inbound on the next heartbeat — so the inbound
term can now outlive the confirm window only while the team is genuinely
still apart, which is the case it was written for.
```

### `explo_planner_node.cpp:13173-13206`

**Documents:** `const auto settle_now = this->now();`

**Cites:** measured

```
ONLY AN APPOINTMENT SETTLES (2026-09-18). The hold is the last clause of
the rendezvous rule — meet at the agreed cell, wait for everyone, "then
wait for a while more so that map is updated" — so it belongs to the
appointment and to nothing else. Charging it to every barrier put it on
PURSUIT's reunions and on the terminal anchor hold too, which costs those
arms rendezvous_settle_sec per manoeuvre for a mechanism they are the
control for: the four-arm design reads as a factorial, and a treatment
leaking into the control arm is exactly the leak that makes it stop
reading as one. Every non-appointment barrier is now treated identically
(no settle) in all four arms, so what is left is a difference between
arms rather than a difference between barriers.
NO CLOCK CONDITION IN THE GATE (2026-09-18). It used to carry
`&& t_now >= 0.0` against missionElapsed(), which meant an unresolvable
mission clock SKIPPED the settle and released the barrier on the spot.
See the member: the settle is a duration, it is measured on the sim
clock this state already reads for everything else, and there is no
longer a state in which it can be silently not applied.
```

### `explo_planner_node.cpp:13326-13346`

**Documents:** `if (!rendezvous_reagree_waiting_) {`

**Cites:** campaign, measured, `ts4`

```
ONLY WHERE THE SETTLE STAGE DID NOT ALREADY ASK, which is what
rendezvous_reagree_waiting_ records. This raise was unconditional and
described as idempotent, and it is not: the request is SPENT by the
adoption that answers it, so raising it again after an appointment's
wait authorises a SECOND derive, and deriveRendezvousProposal mints
t_meet as "now plus the interval" — the instant the team has just
committed to slides forward by however long the release took. The ts4
gen-29 N=2 rendezvous smoke measured it: cell 23 committed twice, at
t+1055s and then 6 s later at t+1061s, off one meeting.
```

### `explo_planner_node.cpp:13414-13440`

**Documents:** `if (appointment_manoeuvre_ && appointment_settle_converted_ &&`

**Cites:** campaign, measured, `ts4`

```
THE CONVERSION IS REVERSIBLE (2026-09-19, generation 28). A walker that
joined the barrier from the road did so on one premise — the team had
settled, so this spot was as good as the cell. When that premise lapses
the premise is all that is gone: the robot is left stopped in open forest,
metres of unfinished walk from the one place the team agreed to be, and
through generation 27 nothing ever sent it the rest of the way. The
ts4 gen-27 N=2 rendezvous smoke cell measured it — both robots converted
mid-drive at 367/369 s, the link died at 396 s and never returned, and
both stood 13.0 m and 7.2 m short of the agreed cell until the 660 s cap:
330 s each, no outcome row, coverage frozen, and the one action that
would have closed the pair (finishing the walk, which ends with them
co-located) was the action the conversion cancelled. Waiting longer could
not fix it, because the wait is deliberately unbounded and the robots were
not where the waiting was supposed to happen.
```

### `explo_planner_node.cpp:13459-13489`

**Documents:** `const double wait_cap =`

**Cites:** campaign, count, measured, `ts4`

```
  appointment  STAY UNTIL EVERYONE IS HERE. The place was agreed by the
               whole team in advance, so the only reason a peer is absent
               is that it has further to drive or noticed the separation
               later — both of which resolve by waiting. (Since
               generation 27 a peer merely MESH-dark at the gathered cell
               no longer spends this patience — the reachable door
               releases that shape — so the peer waited on here is one
               outside the closure itself.) Giving up here
               converts a meeting that was going to happen into a no-show,
               which is what the gen-18 ts4 N=3 cell measured: husky spent
               the full 240 s and walked away from a cell two of the three
               robots had already stood on together. Unbounded by default
               (rendezvous_appointment_wait_sec <= 0).
  pursuit      the mid-run cap. A chase drives to a PREDICTED intercept,
               which can simply be the wrong place, and there is no
               agreement behind it to be kept — so giving up is the right
               response to a bad prediction rather than an abandonment.
  terminal     the field cap, shortened after an escalation (the second
               wait is a confirmation of failure, not a second full vigil).
```

### `explo_planner_node.cpp:13516-13527`

**Documents:** `if (coverage_latched_ && coverage_latch_hold_start_sec_ >= 0.0 &&`

**Cites:** measured

```
Measured from the LATCH, not from state entry: `waited` starts when the
robot entered RETURN_SYNC, which is before it finished, and charging the
hold for that time would cut it short by however long the robot waited
while it was still exploring.
```

### `explo_planner_node.cpp:13560-13569`

**Documents:** `if (rendezvousWaitExpired(waited, wait_cap)) {`

**Cites:** measured

```
A BLOCK THAT REWOUND `waited` TO max(arrival, t_meet) STOOD HERE, and
generation 19 makes it provably a no-op rather than merely unnecessary.
It existed because t_meet was a MEETING time the robot aimed to arrive
before, so a punctual robot began spending its patience while the meeting
was still in the future and could time out ahead of its own appointment
(measured: atlas gave up at lateness_sec = -11.184). Under the countdown
rule t_meet is the DEPARTURE time, so arrival is necessarily at or after
it, `since_meet >= waited` always holds, and the min() could only ever
return `waited`. Keeping it would leave a reader believing the barrier
still has a second clock in it.
```

### `explo_planner_node.cpp:13716-13737`

**Documents:** `if (mission_return_done_) {`

**Cites:** campaign, count, measured, `ts1b`

```
GUARD 1, run-scoped: the mission return already RESOLVED. Measured on
ts1b: 16 DONE->RETURN_HOME re-entries and 16 robot-runs carrying two
mission_complete rows, all in the treatment arm. The request itself is
legitimate — the coverage latch runs off the metrics tick, which measures
in every state, so a robot that ended on barrier-gave-up at t=1189 can
genuinely saturate its map at t=1900 while parked at home. Only the
response is wrong. What that latch stamps is kept: recordExplorationComplete
runs in the caller BEFORE this call and nothing here gates it, so the 16
genuine late exploration endpoints survive; it is the redundant homing leg
— asked of a robot that is already home — that is refused.
```

### `explo_planner_node.cpp:13751-13755`

**Documents:** `if (state_ == State::RETURN_HOME) {`

**Cites:** campaign, `ts1b`

```
GUARD 2, leg-scoped: a homing leg is still IN FLIGHT. The one already
running is strictly further along than the one this call would start.
DEBUG rather than WARN because this fires on ordinary tick-loop re-entry,
where nothing has gone wrong — and it is not the guard the ts1b defect
needed, which is why guard 1 exists above it.
```

### `explo_planner_node.cpp:13870-13875`

**Documents:** `double leg_budget = std::max(`

**Cites:** measured

```
Distance-true budget, same model as the manoeuvre legs (see
startReturnTo). Measured from NOW (elapsed added) because a retry
re-arms this mid-leg: a budget recomputed from the shrinking remaining
distance alone would fall below the already-elapsed time and fire on
the next tick. No min() against the mission cap — the cap check below
runs first every tick, so it bounds the leg regardless.
```

### `explo_planner_node.cpp:13976-13992`

**Documents:** `const float metric = homeApproachMetric();`

**Cites:** seed

```
---- Three independent watchdogs ----
FROZEN: gross metres travelled, the historical test, measurement UNCHANGED
(15 s / 0.2 m, shared with the NAVIGATE paths). It answers "is the
platform physically moving at all?".
APPROACH: remaining distance home. It answers the question the frozen test
structurally cannot — "is any of that movement getting the robot home?" —
and is the whole point of this generation: seed11 orbited at 0.05 m/s for
the full 600 s cap, passing the frozen test in every window, netting
-0.08 m of approach.
They are evaluated together, both windows are advanced whichever fires,
NEAR-HOME STALL: time spent inside the radius where APPROACH is muted. It
answers the third question, "is the robot close and simply not finishing?",
which neither of the others can: a robot circling 2 m from home passes
frozen every window (it accumulates metres) and approach never even runs.
They are evaluated together, both windows are advanced whichever fires,
and `kind` in the log distinguishes them, so each defect stays attributable
to exactly one detector.
```

### `explo_planner_node.cpp:14271-14308`

**Documents:** `void ExploPlannerNode::homeWatchdogFire(const char* kind, float metric,`

**Cites:** count, seed

```
The single decision point for a homing watchdog fire. Graduated, and never
instantly fatal on a first fire: a false positive costs a resend, or at
worst a demotion to the retrace, which is slower but convergent. Only an
exhausted escape budget parks the robot.
  approach in DIRECT, fire 1 -> resend the home goal
  approach in DIRECT, fire 2 -> engage the retrace
  approach in RETRACE       -> escape leg
  frozen, any mode          -> escape leg (a stuck platform will not be
                               unstuck by resending the goal it is already
                               failing to follow — seed16's resend at
                               +106 s changed nothing)
  frozen during an escape   -> abort the leg, back to the retrace
  escape budget exhausted   -> park, with the unchanged result vocabulary
Termination: DIRECT gives at most 2 fires before it becomes RETRACE, every
RETRACE fire spends one of a bounded number of escapes, and an escape leg
can only end in RETRACE.

WHAT ACTUALLY BOUNDS A WEDGED ROBOT — read this before quoting the 600 s
cap. mission_return_max_sec is the outer bound only for a robot that keeps
MOVING without arriving (the seed11 orbit case: it passes every frozen test
and rides the cap to 600 s). A robot that cannot move at all is parked far
sooner, by this ladder rather than by the cap, because `frozen` alone walks
the whole thing:
    15 s  frozen in DIRECT              -> escape 1   (want_escape is true
                                           for frozen in ANY mode)
    30 s  frozen during that escape     -> abort to RETRACE (no escape spent)
    45 s  frozen in RETRACE             -> escape 2
    60 s  abort;  75 s escape 3;  90 s abort
   105 s  frozen in RETRACE, budget spent -> park, "no-progress"
i.e. (2 * return_escape_max_attempts + 1) * progress_window_sec = 105 s at
the campaign's 3 escapes and 15 s window. That is the intended outcome —
a robot that has not moved 0.2 m in any of seven consecutive windows is not
going to, and censoring it at 105 s is both honest and cheaper than 600 s of
sim. It is written down here because the recorded mission-end time of a
censored cell differs by ~8 minutes depending on which bound fired, and
exposure to this ladder is not arm-symmetric (see the nav-failure counts),
so any analysis must treat 105 s and 600 s parks as the same event class and
must not read the difference as a treatment effect.
```

### `explo_planner_node.cpp:14602-14630`

**Documents:** `have_dispatch_predict_ = false;`

**Cites:** measured

```
IT DOES RESIZE THE WATCHDOG, and this comment denied it ("does NOT get to
touch the budget ... the mdp and trail arms differ in where the chase drives
and in nothing else") until 2026-09-18. When the swap fires, `budget` is
recomputed from the intercept route at the swap site below. That is not the
model buying itself chase time — the CEILING is untouched, the same
pursuit_budget_max_sec_ the trail is priced against, and the affordability
test the swap must pass is asked at the same measured rate as the trail's
own refusal. What it prevents is a watchdog sized for one route killing a
drive down a longer one, which would have scored as the model losing on an
endpoint it never reached. The confound the old sentence guarded against is
still guarded against; it is the cap that does it, not the budget.
```

### `explo_planner_node.cpp:14677-14681`

**Documents:** `auto travelSec = [&](double metres) {`

**Cites:** measured

```
P1 / §3.8. The OTHER question: not "when do I give up on this leg" but
"can this chase be done at all". Measured speed, no safety factor — see
pursuit_speed_measured_mps_ for why the watchdog's inflated rate is the
wrong yardstick for a refusal, and why neither correcting that rate nor
raising the cap is the fix.
```

### `explo_planner_node.cpp:14765-14790`

**Documents:** `if (mdp_travel <= pursuit_budget_max_sec_) {`

**Cites:** measured

```
The arithmetic, at nav_speed_estimate 0.15 / safety 3.0 / measured 0.40:
modelSec is 20 s per metre and travelSec is 2.5. In the stale branch
`trail` is exactly [peer_pose], so trail_m is the straight line to the
contact pose, while mdp_trail is [intercept, peer_pose] — so mdp_m >=
trail_m by the triangle inequality, with equality only for an intercept
sitting on that very line. budget is clamp(20*trail_m, 30, 600), which
makes the old test `mdp_m <= trail_m` in the whole mid-range
(trail_m in [1.5, 30] m), `mdp_m <= 1.5 m` below it, and `mdp_m <= 30 m`
above it — against an mdp_m that is >= trail_m > 30 in that last case.
Every branch is unsatisfiable except the degenerate collinear one. The
non-stale branch is the same story with a staleness discount making it
harder still.

It was also the wrong question twice over. `budget` is the WATCHDOG — how
long before this leg is called dead — and P1/§3.8 established that a
refusal must be asked at the measured rate against pursuit_budget_max_sec,
which is exactly what the stale branch's own trail refusal above does.
Asking the intercept the same question on the same yardstick is what
makes the two aims comparable; asking it a harsher one is how the model
was silently disabled.
```

### `explo_planner_node.cpp:14811-14814`

**Documents:** `dispatch_predict_.refused = "intercept exceeds the chase budget";`

**Cites:** stat

```
Report it as a refusal so the dispatch event says the model was asked,
answered, and lost — distinct from the model having nothing to say. The
cell/probability fields stay populated on purpose: a downgrade at
p = 0.8 and one at p = 0.11 are different findings.
```

### `explo_planner_node.cpp:15085-15103`

**Documents:** `const rclcpp::Time fb_now = this->now();`

**Cites:** campaign, count, measured, `gt1`

```
THE SECOND DISPATCH SITE, and the reason the veto at the mid-run trigger
was not enough on its own. gt1's first 11 cells put the trigger-site leak
at 0 fires with the radio up (against 3 in the ungated tl1 arm), and left
exactly one leak standing — here. Measured over both arms, escalation from
an exhausted chase fired with the radio UP 2 times out of 2, and during a
genuine outage 0 times out of 2. That lopsidedness is not bad luck, it is
the mechanism: the trail runs out BECAUSE the peer moved on, and a peer
that moved on has usually come back into contact. So the site that most
reliably needs the veto was the one running without it.
```

### `explo_planner_node.cpp:15412-15480`

**Documents:** `} else if (hb_gap_sec >= team_model_.config().direct_ttl_sec) {`

**Cites:** campaign, count, measured, `ts4`

```
The condition on this block is rendezvousTeamMutual() — EVERY pair in
the fleet in direct contact. At N=2 that is just "the link is up" and
recurs constantly. At N>=3 in this radio regime it holds for a few
seconds at spawn and then never again, because it needs all N(N-1)/2
links up simultaneously and the forest takes them down independently.
So the flag latched true after the FIRST appointment and every later
separation refused with "the agreed pair has already been used in this
outage" — measured in the ts4 N=3 cell as five refusals on bestla and
two on husky, leaving the arm inert from t~170 s to the end of a 604 s
run. 70% of the cell, on the arm the cell exists to measure.
```

### `explo_planner_node.cpp:15691-15736`

**Documents:** `if (!hb_suppress_warned_ && held >= coord_claim_ttl_sec_) {`

**Cites:** banked, count

```
THE EXEMPTION IS EPISODE-SCOPED, NOT INSTANT-SCOPED, and that distinction
is the whole of the 2026-09-18 fix. A first cut tested `state_ !=
WAIT_FOR_MAP` at the WARN site and nothing else, which was wrong by about
one second: the state leaves WAIT_FOR_MAP when the hold expires, but the
heartbeat does not resume until the executor next runs, so there is a
window in which the robot is already in PLAN while `held` is still the
sixty-second hold. A tick landing in that window found a passing state
guard, an unset latch, and held >= TTL, and fired the exact line the gate
existed to prevent — reported as "suppressed 60.1 s in state PLAN". It
was intermittent, which is worse than always: it fired in 2 of 6 banked
gen-22 cells, so a reader who checked one clean cell would have concluded
the gate worked.
```

### `explo_planner_node.cpp:16137-16155`

**Documents:** `if (coord_ && rendezvous_expected_peers_ > 0) {`

**Cites:** measured

```
Both use accountedPeerCount / peerAccounted, the calls the release predicate
and the outcome classifier make, so the CSV cannot disagree with either.
The expected-peers test is here and not left to teamComplete(), which
folds "not configured" into "not complete": planner_util.cpp returns
`expected_peers > 0 && active >= expected`, so with the yaml default of 0
("0 = inert") every row would read a clean, measured-looking 0 and
mean(team_complete) would come out 0.0 on a run where the question was
never asked. -1 is "there was no team to ask about", a third thing, and the
same sentinel discipline the pursuit columns above use. Invisible on the
campaign path -- the harness always passes N-1 -- and live for any bare
`ros2 run`, which is exactly the case with nobody watching.
```

### `explo_planner_node.cpp:16212-16216`

**Documents:** `updateCellWorld(uf, cov_src);`

**Cites:** measured

```
The coarse cell census, taken here for the same reason the milestones are:
it reads the map that `uf` above was just measured on, on this tick. P1's
gate asks whether the census agrees with `uf` as coverage saturates, and
this sim is nondeterministic enough that sampling the two from different
hooks would have them measuring different maps. No-op when disabled.
```

### `explo_planner_node.cpp:16235-16241`

**Documents:** `void ExploPlannerNode::metricsTick() {`

**Cites:** measured

```
Sample the CSV on a periodic timer, in every state. The period is SIM time
(use_sim_time), so at RTF != 1 it is not a wall-clock period; the adaptive
back-off below is the only part measured on a steady wall clock, because it
bounds executor-thread work. Steps only advance through
the explore loop, so without this a robot that spends four minutes in PURSUE
or RETURN_NAV contributes one flat segment between two step rows across the
exact interval the comms experiment measures.
```

### `explo_planner_node.cpp:16282-16287`

**Documents:** `++metrics_rows_written_;`

**Cites:** measured

```
Realised-rate accounting for this sampler, reported in the event log's
run_end. The configured period is still not a guarantee: the gate above
is a WALL-clock deadline (it must be — it bounds executor work), so the
sim-time spacing of rows scales with RTF, and the duty back-off below
stretches it further under load. Measured from the sim stamps of the rows
actually written, which is the quantity an analysis needs.
```

### `explo_planner_node.cpp:16452-16453`

**Documents:** `e.silent_sec = (now - it->second.last_heard).seconds();`

**Cites:** measured

```
The outage this message ends, measured from the last one that preceded
it — NOT from when the sweep noticed, which lags by up to a claim TTL.
```

### `explo_planner_node.cpp:16576-16580`

**Documents:** `e.unknown_fraction = last_unknown_fraction_;`

**Cites:** measured

```
The last measured coverage rather than a fresh measurement: this runs on
the DONE transition and (via the destructor) during shutdown, where
map_cache_ may be mid-teardown and a whole-grid walk is the last thing to
start. fillCommonMetrics refreshes it at the sampling period, so it is at
most one metrics period old.
```

### `explo_planner_node.cpp:17273-17280`

**Documents:** `if (coord_ && coord_->enabled() &&`

**Cites:** banked

```
Hold branch: with a dwell of our own banked and every other ring angle
team-visited or peer-denied, PARK on the dwelled vantage instead of
running the full selection below (see holdDwelledVantage for the run9
thrash this replaces). Placed above the map refresh + flood on purpose:
a holding robot's tick must cost microseconds, not seconds, or the
executor starves the very intent subscription whose claims the hold
decision reads. Quota merge and the per-target timeout stay ABOVE this,
so a hold can always end.
```

### `explo_planner_node.cpp:17473-17477`

**Documents:** `pending_sep_peer_dist_m_    = -1.0f;`

**Cites:** measured

```
The separation term does not run on the exploit path — vantages are
chosen by sightline, not by utility — so these are reset to their
"nothing measured" values rather than left holding whatever the last
exploration tick saw. A stale peer distance on an exploit row would be
read as a measurement of where this robot was sent, and it is not.
```

### `explo_planner_node.cpp:18179-18183`

**Documents:** `std::vector<double> cell_u, cell_ff;`

**Cites:** measured, stat

```
Unknown fractions of the MEASURED cells, gathered here and reduced below.
Only cells that cleared min_observed_columns go in: an unmeasured cell
reports unknownFraction() == 1 by construction, and letting those into the
distribution would make the median a statement about how far the grid
overhangs the ROI rather than about how well the seen ground is mapped.
```

### `explo_planner_node.cpp:18222-18226`

**Documents:** `auto pct = [&cell_u](double q) {`

**Cites:** measured

```
Nearest-rank percentiles, so every reported value is a real cell's
measurement rather than an interpolation between two of them. With a
handful of measured cells an interpolated p10 can sit below every cell
on the map, which is exactly the wrong answer to "is the threshold
reachable".
```

### `explo_planner_node.cpp:18425-18475`

**Documents:** `const State tour_state = (state_ == State::PROXIMITY_HOLD)`

**Cites:** count

```
P3/P6: this robot's current route, for the peers' interception model. The
wire type is uint16 and a cell id is an int, so ids are bounds-checked
rather than cast — an id that does not survive the round trip would name a
different cell on the receiver, and a chase aimed at it is aimed at ground
nobody chose. Out-of-range TRUNCATES the tour rather than skipping the
offending entry: the consumer walks this as an ordered route and a route
with a hole in it is a route through a cell the producer never planned to
visit. It cannot happen on any grid this planner builds (the ROI would
have to exceed 65535 cells) and it is checked anyway, because the failure
is silent and the check is free.
```

### `explo_planner_node.cpp:18683-18703`

**Documents:** `std::map<int, double> pre_known_age;`

**Cites:** banked, count

```
Through v7 the row reported team_model_.lastKnownAgeSec(sender) taken in
the second pass, which is structurally 0.0: observe() sets the sender's
last_known_sec to this same `t`, and a row only exists for a sender. It
read 0.0 on 757,677 of 757,677 rows across the banked campaigns — a column
that could not have reported anything else.
```

---

## Provenance and limits

Extraction is mechanical: a comment block qualifies if any paragraph in it
contains a count of the form *N of M*, a campaign token, a ratio over
cells or runs, a statistic (`n =`, `p =`, `rho =`, a median), the word
*measured*, a count of cells/runs/fires/rows, the word *banked*, or a seed
reference. Only the qualifying paragraphs are reproduced, not the whole
block — the surrounding mechanism description stays in the source.

Two limits worth stating plainly:

1. **Line numbers drift.** They are valid against the revision named
   above and nothing else. A citation without a revision is not a citation.
2. **Nothing here has been re-verified against the campaign corpus.** This
   document records what the source *claims*, not what the data shows. The
   one claim in the package that has been checked end-to-end turned out to
   be false.
