# Method: an information-theoretic exploration planner with perceptive exploitation

*Reference description of `explo_planner`, written for reuse in a paper. Every
tunable number quoted is the shipped default in
[`explo_planner/config/shared_params.yaml`](../explo_planner/config/shared_params.yaml).
Numbers not in that file (the 10 Hz tick, ε in §7, the Nav2 recovery rule in §8)
are fixed in the code.*

---

## 1. Overview

The planner repeatedly answers one question: *given what the robot knows, where
should it look next?* It answers in two senses, and the split between them is
the core of the design.

During **exploration** the objective is unknown, so the planner maximises
expected information. It proposes viewpoints, simulates what the sensor would
see from each against the current map, and drives to the one that buys the most
new knowledge per metre travelled. During **exploitation** the objective is
known: a specific tree trunk must be imaged from several sides. The planner then
drops information gain and optimises prescribed angular coverage. It circles the
trunk at a fixed set of viewpoints, checks each for an unobstructed sightline,
and holds still at each long enough for the sensors to capture a clean,
overlapping view.

Both modes share one state machine, one goal interface to the navigator, one
travel-cost model and one multi-robot coordination layer. Exploitation is an
overlay, not a separate system. A tree target arriving on a shared topic
preempts exploration, and the planner returns to exploration when the target
queue drains. The exploration/exploitation split is therefore an experimental
variable: with the same binary and parameters, conditions differ only in
whether targets are released.

The planner neither commands velocity nor fuses sensor data. It publishes a goal
pose and lets the navigation stack plan the trajectory. Its output is a sequence
of *where the robot stood and when it stood still*, which is what the offline
reconstruction consumes.

## 2. Inputs and world representation

The planner reads a **fused probabilistic occupancy map**, produced by the
SCovox mapping stack and merged across the team. Each voxel carries a Beta
distribution `Beta(a_occ, a_free)` over its occupancy probability. The two
parameters count evidence: how often the voxel has been observed occupied and
free. Their ratio gives the mean occupancy `p = a_occ / (a_occ + a_free)`; their
sum gives the confidence in it. An unobserved voxel is absent from the map and
is treated as the uniform prior `Beta(1, 1)`: occupancy 0.5, no confidence.

The information measure (§5) relies on this split between *uncertain* and
*unobserved*, which is why a Beta map is used rather than a log-odds grid. A
log-odds cell at p = 0.5 may be unseen, or seen a hundred times with
contradictory evidence. The Beta map separates the two, and only the first is
worth visiting.

The robot's pose comes from the transform tree (`map` → the robot's base frame),
so the planner shares a frame with the map and the navigator. A 2D inflated
occupancy grid can optionally be used for obstacle and reachability filtering
(§6). It is **disabled by default**. The real-robot configuration enables it,
fed from a LiDAR traversability map ([field_setup.md](field_setup.md),
"Traversability map").

The planner clips the map to a region of interest (ROI): a rectangle in the
horizontal plane, which may be rotated relative to the map frame, and a vertical
band. In **terrain-relative mode** (the shipped default) the band is measured
from the robot's own altitude, not fixed in world coordinates, so it rides with
the robot across sloping ground. Candidate viewpoints sit on the locally
detected ground surface plus a fixed sensor clearance (0.3 m), not at a constant
absolute height. Without this, a survey area with more than a few metres of
relief pushes the band off the ground, and the planner scores viewpoints in
mid-air or underground.

## 3. The planning cycle

The planner runs a state machine at 10 Hz:

```
WAIT_FOR_MAP → PLAN → NAVIGATE → INTEGRATE → LOG_STEP → PLAN … → DONE
                         │  ▲
                 PROXIMITY_HOLD           (yield while a peer drives past)
exploit sub-loop:  EXPLOIT_PLAN → NAVIGATE → EXPLOIT_DWELL → LOG_STEP → …
rendezvous:        RETURN_NAV → RETURN_SYNC  (drive home, hold for the team)
```

In exploration the cycle is:

- **PLAN**: select the next viewpoint (§4–§7) and publish it as a goal.
- **NAVIGATE**: monitor the drive; detect arrival, timeout or stall (§8).
- **INTEGRATE**: hold still for 2 s so the mapper folds in observations from the
  new pose before they are scored.
- **LOG_STEP**: write one row of per-step metrics (§13) and return to PLAN.

Exploitation replaces PLAN and INTEGRATE with **EXPLOIT_PLAN** and
**EXPLOIT_DWELL**, and reuses NAVIGATE and LOG_STEP. **RETURN_NAV** and
**RETURN_SYNC** implement the multi-robot rendezvous (§10.2), and
**PROXIMITY_HOLD** the coordinated stop (§10.3). A run ends in **DONE**, which
either shuts the node down or (the shipped default) leaves it idling, ready to
service targets released later.

A *step* is one full cycle. The step count, not wall-clock time, is the
planner's own budget. Runs end on coverage saturation (§9), on the step budget,
or on the experiment's wall clock, whichever comes first.

## 4. Candidate generation

Each planning cycle proposes a set of candidate viewpoints. A candidate is a
position and a heading, because what a sensor sees depends on both.

**Frontier candidates** are the shipped candidate set. A frontier voxel is a
free voxel next to unobserved space: the boundary of what is known. Frontier
voxels anywhere in the ROI are binned into 5 m cubes, and each bin's centroid
becomes a candidate, heading along the bearing from the robot to the centroid.
Frontiers are what let the planner commit to crossing the map. A purely local
candidate set finishes one area and then has nothing worth visiting, because
every nearby viewpoint is already known.

**Local candidates** can be added on a polar grid centred on the robot: three
distance rings (2, 5 and 8 m), eight bearings per ring and four headings at each
position, 96 candidates in total. They give dense, reliably reachable options
near the robot. The polar grid is **disabled in the shipped configuration**.

Candidates outside the ROI are dropped, as are those in occupied space when a
map is available to check against.

## 5. Expected information gain

Each candidate is scored by simulating what the sensor would see from it. The
planner casts a grid of 16 × 12 rays over a 60° × 45° field of view, from 0.3 m
to 10 m. Each ray is walked through the map, accumulating a per-voxel score, and
stops at the first confidently occupied voxel (p ≥ 0.7) to simulate occlusion.
Rays are first clipped to the ROI, so space the map does not cover contributes
nothing instead of scoring as maximally uncertain. Without the clip the planner
is biased toward staring over the edge of its own map.

The per-voxel score is the **expected information gain** (EIG): the mutual
information between the voxel's unknown occupancy and the next observation of
it,

```
EIG(v) = H(p) − E[H(y | θ)]
       = H(p) − [ ψ(s+1) − p·ψ(a+1) − (1−p)·ψ(b+1) ]
```

where `a = a_occ`, `b = a_free`, `s = a + b`, `p = a/s`, `H` is the binary
Shannon entropy and `ψ` is the digamma function. The first term is the
uncertainty of the *prediction* of the next measurement. The second is the
uncertainty left in that measurement once the true occupancy is known. Their
difference is how much of the prediction's uncertainty an observation would
resolve: evidence gained, not noise re-measured.

This fixes what a naive entropy measure gets wrong. A voxel observed a hundred
times as ambiguous has high entropy but near-zero EIG, because another look will
not settle it. An unobserved voxel sits at the `Beta(1,1)` prior and yields the
maximum. The planner is pulled toward new space rather than toward whatever is
noisiest. Noise-seeking is the failure mode of entropy-driven next-best-view
planning in vegetation, where partly transmissive foliage produces large volumes
of permanently ambiguous occupancy.

Two variants exist, and neither is used in the shipped configuration:

- A **ray-marginalised** formulation following Asgharivaskasi and Atanasov
  (T-RO 2023) weights each voxel by the probability that the ray reaches it, so
  occluded volume behind a wall stops inflating a score. It is implemented in
  the ray evaluator, but the planner node has no parameter that selects it.
- A **trajectory-scoring** variant sums the score over poses sampled every
  1.5 m along the path to a candidate, instead of at its endpoint alone. It
  needs the 2D grid's path, multiplies the scoring cost by the path length, and
  is the one setting that can push a planning cycle past its time budget.

## 6. Travel cost

Information alone would send the robot across the map for a slightly better
view, so each candidate is weighed against the cost of reaching it.

When the 2D grid is available, the cost is the true path length. One
8-connected Dijkstra flood runs outward from the robot's cell per planning
cycle, and each candidate is then a constant-time lookup. The flood is bounded
at 2 m beyond the outer candidate ring (10 m), which keeps it under a
millisecond. Candidates beyond the bound, and candidates in free pockets sealed
off by obstacles, take infinite cost. The flood therefore doubles as a
reachability test that rejects such goals before they are published, rather than
after the navigator has spent a minute failing to reach them. Exploitation
floods without a bound, because a tree target may be released while the robot is
on the far side of the map.

Without the 2D grid (the default), the cost is straight-line distance and there
is no reachability filtering. Obstacle avoidance is left to the navigator, and
unreachable goals are caught after the fact by the checks and blacklist of §8.

## 7. Selection

Candidates are ranked by information per unit distance:

```
U(c) = EIG(c) / (ε + cost(c)),    ε = 0.1 m
```

The form has no weight to tune between information and distance. A candidate
twice as far must be twice as informative to win; ε only prevents division by
zero for candidates at the robot's feet. Unreachable candidates take `U = −∞`
and sort last.

The planner walks the ranking and takes the first candidate that passes every
filter:

1. not within arrival tolerance of the robot (a goal already reached wastes a
   step);
2. on free, reachable ground, when a 2D grid exists to check;
3. not within 2 m of a goal that failed in the last 60 s (§8);
4. not contested by a teammate with a better claim (§10.1).

The winner is published as a goal pose and broadcast as an intent.

## 8. Execution and failure recovery

The planner is optimistic about navigation and explicit about giving up.

**Time budget.** Each drive gets `clamp(d / v̂ × k, 30 s, 180 s)`, with an
assumed speed `v̂` of 0.15 m/s and a safety factor `k` of 3: a 2 m hop gets
40 s, a 15 m crossing 180 s. `d` is the planned path length in metres where a 2D
grid exists, not the straight line, so a goal 5 m away behind a 15 m detour is
not failed for being slow.

**No-progress watchdog.** At the end of each 15 s window, the goal fails at once
if the robot is less than 0.2 m (straight line) from where it stood when the
window opened. This catches a wedged robot, which the time budget alone would
take minutes to catch. Net displacement is used rather than path walked, because
pose jitter and rocking in place accumulate path on a robot that goes nowhere.
Single-tick pose jumps above 1 m are treated as localisation corrections, not
motion, so a relocalisation can neither spoof the watchdog nor inflate the
logged distance.

**Navigator verdicts.** When the navigator is Nav2, the planner watches the
`NavigateToPose` action's status, feedback and (on Jazzy) the error code of an
abort. A goal Nav2 aborts twice in a row with the robot standing still is failed
at once rather than left to the budget; the planner's periodic keep-alive
re-send of the goal gives Nav2 its second try. Such a goal is typically outside
Nav2's costmap, or has no path. One abort is enough when Nav2 had already run at
least two recovery behaviours within 1 m of where it aborts (a wedged robot),
because a re-send would only repeat them. Everything else Nav2 reports is
recorded (§13), not acted on.

**Arrival.** Position and heading must both be within tolerance (0.4 m,
0.4 rad), because a viewpoint's value depends on where the sensor points. The
in-place rotation after the position is reached has its own 15 s deadline,
separate from the drive budget, so a robot that arrives late in its budget is
not failed while standing on the goal. The planner's tolerances must stay
strictly looser than the navigator's own goal checker. Otherwise the navigator
can stop inside its tolerance but outside the planner's, and the planner never
registers arrival.

**Blacklist.** A goal that fails for any reason is blacklisted: a 2 m disc
around it is rejected for 60 s. Without this, a goal behind an unmapped obstacle
is re-selected on the very next cycle, indefinitely. Entries expire, so a
location that becomes reachable as the map grows is retried.

## 9. Termination

Exploration ends when the ROI is saturated: the unknown fraction is below 5 % on
three consecutive measurements, each on a new step or a newly fused map.
Requiring a streak stops a single measurement gap from ending a run early.

Coverage is a **2.5D column statistic** over the fused 3D map: the fraction of
ground-plane columns in the ROI with no observed voxel at any height within the
band. Volumetric unknowns are deliberately ignored. Trunk interiors and canopy
shadow are never observable and would put a permanent floor under a true 3D
unknown fraction, so a run would never end. The column measure asks the
operational question instead: *has the robot seen anything at all in this part
of the area?*

The measure depends on the ROI. Columns inside it that no robot can reach never
clear and put a floor under the unknown fraction, so the threshold must be
calibrated above that floor for each site.

## 10. Multi-robot operation

Each robot runs its own planner against its own copy of the fused map. There is
no central allocator and no leader. Three independent mechanisms coordinate the
team, and each reduces to single-robot behaviour when the robot is alone.

### 10.1 Goal deconfliction (MinPos)

Each robot broadcasts an **intent** at 1 Hz: its selected goal, its own pose, a
claimed disc radius (the sensor range, 10 m) and a time-to-live (TTL, 5 s).
Peers keep the latest intent per robot and drop it when the TTL lapses, so a
robot that crashes or leaves radio range releases its claims automatically.
Exploitation claims are kept for a grace period past expiry, for vetoes only
(§11, claim grace).

When two robots want the same region, a pairwise form of the **MinPos** rule
(Bautin, Simonin and Charpillet, ICIRA 2012) decides: the robot closer to the
contested viewpoint keeps it, and the other rejects the candidate and takes the
next one on its own ranking. Equal distances go to the lexicographically smaller
robot name. The comparison is total and antisymmetric, so exactly one robot
yields per conflict: never both, and never neither.

Claim expiry is timed from **local receipt**, never from the sender's timestamp.
Robot clocks in the field are not synchronised (offsets of hours have been
observed), and an expiry computed from the sender's clock makes claims either
immortal or stillborn, depending on the sign of the offset.

### 10.2 Rendezvous reconnection

A robot that loses radio contact keeps exploring alone. The question is what it
should do when it *finishes*. Stopping is wrong: its teammate may still be
exploring, and their maps have not merged.

Instead, a robot that runs out of goals while a teammate is out of contact
drives back to the **mission start point** (its pose when planning began) and
waits there until the whole team is back in contact. It broadcasts its presence
throughout, so arriving teammates can count it. The team launches inside one
comms bubble, so every robot parked at its own start pose is in range of the
others by construction: one deterministic meeting point. Sending each robot
instead to where it last heard a teammate fails, because two robots that lost
contact in different places then wait in different places, possibly never back
in range.

Once reconnected, the robot re-plans against the merged map. If the merge
revealed new frontiers, the team disperses again (MinPos splits them); if not,
everyone finishes together. A robot that finishes with the team already present
also drives home before stopping, so the mission ends with the team parked where
it launched.

The barrier enforces an invariant: **a robot can only finish when the whole team
is present and the merged map is saturated**, so no robot quits while a
teammate is still working. Both legs are bounded for field use. The wait at home
times out after 600 s, for a teammate that has died; a robot that gives up still
parks and beacons its presence, so a late teammate can find it. A return drive
that runs out of time (its budget ceiling is 600 s, not 180 s) or stops making
progress waits where it is. The barrier takes priority over exploitation: open
tree targets are stood down for the drive home and resumed after reconnection,
because a robot detouring to inspect trees would leave its teammate waiting
indefinitely.

### 10.3 Coordinated proximity stop

MinPos deconflicts *goals*, not *paths*. Two robots holding different goals can
still be routed across each other: in one field-trial goal plan the two robots'
commanded routes cross, with a closest approach of 0 m.

While driving, each robot therefore watches its teammates' positions and yields
when a higher-priority teammate is moving nearby. Right of way goes to the
lexicographically smaller robot name, the same total order MinPos uses. The
decision is computed **from identifiers alone**, with no geometry. Both robots
of a pair always agree on who yields, even if their estimates of each other's
positions disagree, so the pair can neither deadlock (both stopping) nor collide
through mutual assumption (both proceeding).

The yielding robot cancels its navigation goal *and* publishes a holding goal at
its own position. The redundancy is deliberate: the cancel is confirmed only
asynchronously, and the zero-travel goal covers a cancel that is lost or
rejected. The robot holds when the peer comes within 5 m and resumes its
original goal once the peer is beyond 6 m. The gap is hysteresis, so a peer
skirting the threshold does not make the robot chatter between states.

Two release conditions prevent deadlock. A peer that has moved less than 0.3 m
in 10 s counts as **parked** and is no longer held against: a stationary robot
is an ordinary obstacle that the navigator's costmap routes around, and holding
against one deadlocks (a teammate waiting at home for the rendezvous would
otherwise stop the team forever). This release has a 1.5 m floor: a peer parked
closer keeps the hold, since "it stopped" is no licence to drive closer still.
As a final escape, the drive resumes after 120 s regardless. Held time is
banked, not refunded, so repeated holds cannot grant one goal unbounded driving
time.

The 5 m trigger is sized against the **reaction budget**, not the nominal
separation. Peer pose age, the planner's tick, cancel propagation through the
navigator and braking together exceed a second. Only the yielding robot slows,
so the pair keeps closing at the peer's speed throughout. At field speeds a 3 m
trigger is consumed entirely by that latency.

## 11. Perceptive exploitation

When a tree target arrives (a trunk centre, an estimated radius and an
identifier), the planner preempts exploration, even mid-drive, and switches to
inspecting it. Targets arrive on a shared topic as a single message type. The
current experiments publish a preselected list on a timed schedule; a live
detector can publish the same message with no change to the planner. That seam
is what makes exploitation timing an experimental variable rather than a
property of the perception stack.

**Vantage generation.** Three viewpoints are placed evenly around the trunk
(120° apart) on a standoff circle, each facing the trunk axis. The standoff is
the trunk radius plus 2 m, clamped so the trunk surface stays beyond the
sensor's minimum range and the axis within its maximum. Ring positions are
deterministic functions of the trunk centre and radius, so index *k* names the
same physical pose on every robot, which makes cooperative capture possible
(below). The ring starts at 30°. With a 0° start, the first vantage of every
ring points straight down a plantation row and the other two land in the
neighbouring canopy, where they fail together.

**Validation.** Each vantage is re-checked every planning cycle, so an angle
blocked earlier can open up as the map improves. A vantage must lie in the ROI,
on free and reachable ground when a 2D grid exists to check, and must have a
**clear line of sight** to the trunk. A ray marched from the vantage toward the
trunk axis, at the vantage's own measurement height, must hit no confidently
occupied voxel before it reaches the trunk surface. Voxels at or inside that
surface are the trunk itself, an expected hit rather than an occluder.
Unobserved space never blocks: a sightline through unmapped volume is unknown,
not known-bad.

**Execution.** The planner drives to the nearest valid, unvisited vantage and
**dwells** there for 8 s. No goal is re-sent during the dwell. A re-send is a
fresh navigation request, and the navigator emits at least one velocity command
even for a goal already satisfied, which would put a rotation in the middle of
the capture the dwell exists to take. With dwell synchronisation on, the dwell
clock does not start until the team is staged on the ring (below). When the
dwell ends, the sightline is re-checked from the pose the robot actually settled
at, and that verdict, not the one from selection time, is credited and logged.

**Completion.** A target succeeds once three clear-sightline dwells are
credited. It closes as *partial* if the per-target budget (300 s since the last
progress event) expires first, which guarantees that one bad tree cannot consume
the mission. Progress events (activation, arrival at an approach waypoint, a
completed dwell) re-arm the budget. Proximity-hold time is refunded, so yielding
to a teammate never costs a target.

**Approach fallback.** If no vantage is currently selectable (the ring is
unmapped, occluded or unreachable), the planner does not abandon the target. It
drives toward the trunk along the line from the robot, to the point closest to
the trunk (but no nearer than the standoff) that is in the ROI, reachable and
not blacklisted. It maps the area en route and re-plans the ring on arrival.
"The ring is not visible from here" thus becomes an information-gathering action
rather than a failure. Termination is preserved because each approach must end
at least 0.8 m (twice the arrival tolerance) nearer the trunk than the robot
already is.

**Cooperative capture.** When two robots work the same trunk, each broadcasts a
bitmask of the ring indices it has captured with a clear sightline. Peers merge
the union into their own record of that target, so the three-vantage quota is a
**team** quota: the ring is covered once, cooperatively, and the trunk closes for
everyone. Because indices name canonical poses and the mask is cumulative state
rather than an event, merges are idempotent and a lost message loses no credit.

**Vantage claims.** Splitting a ring's angles across the team reuses the intent
machinery of §10.1 but not its rules. Vantage claims carry their own small disc
(0.75 m, the tolerance within which a vantage counts as visited). The
exploration-scale disc would cover the whole ring, and one robot would veto the
trunk instead of the two splitting its angles. Three rules decide whether a
candidate vantage is admitted. Which rule applies depends on the *kind* of peer
claim covering it, not on geometry:

1. A peer's exploitation claim on the **same trunk** vetoes the angle
   unconditionally, with no distance contest. A robot standing at the trunk
   after one dwell is, by construction, closer to every remaining vantage than
   a teammate partway through its drive to one. Letting distance decide would
   take the angle from the robot already committed to it and send both to one
   point. What a mid-drive peer has *left* is cost-to-go, and its claim already
   encodes that commitment.
2. Any other claim (an exploration claim, or an exploitation claim on a
   different trunk) keeps the ordinary MinPos distance contest.
3. Peers that are **staged** (standing on a vantage of this trunk, see below)
   are contested by position *before selection*, even though they hold no claim
   on the contested angle yet. This settles a race that happens before any
   claim exists. When the dwell barrier releases, every parked robot re-plans
   within milliseconds, before the 1 Hz claim exchange can separate them, and
   with one angle left all of them pick it (in one run, two robots 3 ms apart,
   ending in a collision). A staged peer is stationary, so its broadcast pose is
   current and both robots evaluate the same antisymmetric comparison: exactly
   one admits the angle, with no message exchanged. Driving peers are exempt.
   Their broadcast pose is up to a heartbeat stale, and a robot approaching from
   afar is farther from *every* angle on the ring, so contesting it by position
   would lock it out of the trunk and serialise what the barrier exists to
   parallelise.

The backstop for the remaining race (two robots that each believed they had won)
is in the drive itself. A robot that sees, mid-drive, a peer's claim on its own
goal re-runs the comparison, and the loser yields: it cancels its navigation
goal and brakes rather than driving on into the winner.

**Synchronised dwells.** By default the team captures a trunk
*simultaneously*. A robot that reaches its vantage holds there, dwell clock
stopped, until every live peer claiming the same trunk is also standing on its
own angle. The sensors then record one trunk state, not a sequence of visits
minutes apart. Staging is **declared by the producer**, not inferred: each
exploitation claim carries a flag that its sender sets on entering the dwell,
after both arrival and the post-arrival rotation have settled. Position alone
cannot tell "on the vantage" from "still rotating beside it" (about 5 s too
early, which starts the team's dwells out of step) or from "parked on an
approach waypoint". The wait is unbounded by default (a deadline can be set) but
releases when a peer's claim ages out on its raw TTL, so a teammate that dies on
approach frees the dwelling robot in about 5 s. The barrier is also the
experiment's A/B lever: with it disabled, the first robot to arrive dwells alone
and can close the quota before its teammate lands.

**Claim grace.** The claim table is maintained on the receiving side by a
subscription that shares a single-threaded executor with planning. Heavy
re-planning (retrying a blocked ring at 10 Hz, with multi-second map refreshes)
can starve it for longer than the 5 s TTL while the sender still heartbeats at
1 Hz. The winner's claim then drops out of the loser's table and rule 1 goes
blind: the parked loser re-selects the angle its teammate is driving to, yields
it back on the next heartbeat, and repeats, drifting off its own pose each time.
Exploitation claims are therefore kept for a **grace period** of 10 s (twice the
TTL) past expiry, during which they still veto and still lose contests. The
grace applies to vetoes only. Everything that infers *liveness* from a claim
keeps the strict TTL (the dwell barrier's release, the rendezvous presence count
and the exploration MinPos contest), so a robot that is really gone still
releases the team in about 5 s.

**Final-angle hold.** With three angles and two robots, one robot ends the last
round with its own dwell banked and every remaining angle either team-captured
or under a teammate's claim. That robot stops planning. It parks on the exact
pose it dwelled from (position *and* heading) and keeps its staged claim
beating, so the teammate's barrier reads it as present, until the merged team
mask closes the trunk. The hold is re-evaluated every cycle with the same veto
rules as selection, so it dissolves by itself if the teammate's claim dies (the
survivor takes the remaining angle) or the target times out. Holding cycles skip
the map refresh and cost microseconds, which also relieves the executor pressure
that made the grace window necessary.

## 12. Coupling to the mapper

Exploitation also drives map resolution. When a target is released, each robot
registers a **refinement region**, a vertical cylinder around the trunk, with
its own mapping node, and unregisters it when the target closes. Inside the
cylinder the mapper additionally fuses observations into a finer-resolution
lattice than the survey map. Fine detail is spent only where inspection
happens, which keeps the global map affordable to store and to transmit between
robots. Regions of targets that never completed are unregistered when the
planner shuts down, so a run cannot leave the mapper refining trunks
indefinitely.

## 13. Instrumentation

One CSV row is written per step. It records:

- the step index and timestamp;
- observed-voxel and frontier counts, and the map's mean information, entropy
  and variance;
- distance travelled;
- the selected candidate's information gain, path cost and utility, with the
  means over all candidates that cycle, so a selection can be attributed to
  either term;
- planning time;
- the number of active peers, and the candidates rejected by deconfliction and
  by unreachability;
- cumulative proximity-hold count and held seconds;
- for exploitation rows: the target, the vantage index, how many vantages were
  valid, whether the sightline was clear, and the dwell time achieved;
- for the goal that ended since the previous row: how it ended, how long it
  took, the path length its budget was built from, and Nav2's aborts,
  recoveries and last error code on it.

The columns make behaviour reconstructable without the logs: whether a robot
was pulled off course by a teammate, whether a trunk closed because it was
captured or because it timed out, and whether a viewpoint was chosen for its
information or merely for its proximity. Column names are listed in
[ros_api.md §7](ros_api.md#7-the-metrics-csv).

## 14. Assumptions and limitations

- **Ground robot.** Distances, deconfliction discs and sightline tests are
  computed in the horizontal plane. Height is handled by snapping viewpoints to
  the local ground, not by planning in three dimensions. The vantage ring is a
  single horizontal circle, with no height-varying inspection pattern.
- **Rectangular ROI.** The ROI is a rectangle, optionally rotated. A survey area
  of any other shape must be entered as an enclosing rectangle. This admits
  candidates outside the true area and, because the same rectangle defines the
  coverage measure, raises the floor under the unknown fraction (§9).
- **Delegated obstacle avoidance.** In the default configuration the planner
  does no 2D reachability filtering. It learns that a goal was unreachable when
  Nav2 gives it up or, with no Nav2 to watch, when the budget or watchdog fires
  (§8), and recovers by blacklisting rather than by prediction.
- **Coordination, not safety.** The proximity stop is best-effort coordination,
  not a certified safety function. It needs live peer data, both planners
  running and the navigator honouring the cancel. In the field the crewed manual
  stop remains the hard backstop. In simulation runs the navigator exposed no
  cancellable action, so the holding goal alone stopped the robot; the cancel
  path is untested outside the field stack.
- **Positioning, not reconstruction.** Whether the captured views suffice for
  the downstream measurement is decided offline. The planner's success
  criterion (three clear-sightline dwells) is a geometric proxy for that, not a
  measurement of it.

Specific known issues are tracked in
[explo_planner/doc/limitations.md](../explo_planner/doc/limitations.md).

---

### Key parameters

| Parameter | Default | Role |
| --- | --- | --- |
| Frontier cluster size | 5 m | Long-range candidate granularity |
| Polar candidates: rings / bearings / headings | 3 / 8 / 4, **off** | 96 local candidates per cycle when enabled |
| Polar candidate radii | 2–8 m | Local viewpoint spacing |
| Ground clearance | 0.3 m | Viewpoint height above local ground (terrain mode) |
| Sensor model | 60° × 45°, 0.3–10 m, 16 × 12 rays | Simulated field of view for scoring |
| Occlusion threshold | p ≥ 0.7 | Ray stops at confidently occupied voxels |
| Utility ε | 0.1 m (fixed) | Division guard in `EIG / (ε + cost)` |
| Nav budget | `clamp(d/0.15 × 3, 30, 180)` s | Distance-scaled drive timeout |
| No-progress watchdog | 0.2 m net per 15 s window | Wedged-robot detection |
| Nav2 give-up | 2 aborts in a row, robot still; 1 after ≥ 2 recoveries within 1 m | Ends goals Nav2 rejects or cannot reach |
| Arrival tolerance | 0.4 m, 0.4 rad; 15 s to rotate | Must be looser than the navigator's own checker |
| Failed-goal blacklist | 2 m for 60 s | Prevents re-selecting failed goals |
| Coverage termination | < 5 % unknown, 3 consecutive measurements | Saturation criterion |
| MinPos claim | 10 m disc, 5 s TTL, 1 Hz | Goal deconfliction |
| Rendezvous wait / return ceiling | 600 s / 600 s | Bounds both rendezvous legs |
| Proximity hold / resume | 5 m / 6 m | Coordinated yield with hysteresis |
| Parked release / floor / max hold | < 0.3 m in 10 s / 1.5 m / 120 s | Deadlock avoidance and its limits |
| Vantages per trunk | 3 at 120°, 30° start | Prescribed angular coverage |
| Vantage standoff | trunk radius + 2 m | Clamped to the sensor envelope |
| Dwell time | 8 s | Capture window per vantage |
| Per-target budget | 300 s since last progress | Bounds time spent on one trunk |
| Vantage claim disc | 0.75 m (visited tolerance) | Splits one ring's angles across the team |
| Exploit claim grace | 10 s (2 × TTL) | Veto-side retention across delivery gaps |
| Dwell synchronisation | on, unbounded wait | Team dwells start together; strict-TTL release |
