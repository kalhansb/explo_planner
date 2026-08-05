# Method: an information-theoretic exploration planner with perceptive exploitation

*Reference description of `explo_planner`, written for reuse in a paper. Every
number quoted is the shipped default in `config/shared_params.yaml`.*

---

## 1. Overview

The planner answers one question repeatedly: *given what the robot currently
knows about the world, where should it look next?* It answers it in two
different senses, and the distinction between them is the core of the design.

During **exploration** the objective is unknown, so the planner maximises
expected information: it proposes viewpoints, simulates what a sensor would see
from each one against the current map, and drives to the viewpoint that buys the
most new knowledge per metre travelled. During **exploitation** the objective is
known — a specific tree trunk must be imaged from several sides — so the
planner abandons information gain entirely and optimises prescribed angular
coverage instead: it circles the trunk at a fixed set of viewpoints, checks each
for an unobstructed sightline, and holds still at each one long enough for the
sensors to capture a clean, overlapping view.

The two modes share one state machine, one goal interface to the navigator, one
travel-cost model and one multi-robot coordination layer. Exploitation is an
overlay, not a separate system: a tree target arriving on a shared topic
preempts exploration, and the planner returns to exploration when the target
queue drains. This matters because it makes the exploration/exploitation split
an experimental variable — the same binary, the same parameters, and the only
difference between conditions is whether targets are ever released.

The planner is deliberately thin about actuation. It publishes a goal pose and
lets the navigation stack solve for a trajectory; it never commands velocity. It
also does not fuse sensor data. Its output is a sequence of *where the robot
stood and when it stood still*, which is what the offline reconstruction
consumes.

## 2. Inputs and world representation

The planner reads a **fused probabilistic occupancy map**, produced by the
SCovox mapping stack and merged across the team before it reaches the planner.
Each voxel carries a Beta distribution `Beta(a_occ, a_free)` over its occupancy
probability rather than a single number. The two parameters count evidence: how
many times the voxel has been observed as occupied and as free. Their ratio
gives the mean occupancy `p = a_occ / (a_occ + a_free)`, and their sum gives the
confidence in that estimate. A voxel that has never been observed is absent from
the map and is treated as the uniform prior `Beta(1, 1)` — occupancy 0.5, with
no confidence at all.

This distinction between *uncertain* and *unobserved* is what the planner's
information measure exploits, and it is the reason a Beta map is used rather
than the more common log-odds grid. A log-odds cell at p = 0.5 could be a cell
nobody has looked at or a cell seen a hundred times with contradictory
evidence; the Beta map separates the two, and only the first is worth visiting.

Two further inputs are required: the robot's pose, read from the transform tree
(`map → base_link`) so that the planner shares a frame with the map and the
navigator; and, optionally, a 2D inflated occupancy grid used for obstacle and
reachability filtering. The 2D grid is **disabled by default**. When it is
absent the planner falls back to straight-line travel costs and delegates
obstacle avoidance entirely to the navigator, which is the configuration used in
the field, where no such grid is published.

The planner clips the map to a region of interest — an axis-aligned box in the
horizontal plane and a vertical band. In **terrain-relative mode** (the field
default) the vertical band is measured relative to the robot's own altitude
rather than fixed in world coordinates, so it rides with the robot across
sloping ground; candidate viewpoints are then placed on the locally detected
ground surface plus a fixed sensor clearance rather than at a constant absolute
height. Without this, a survey area with more than a few metres of relief pushes
the map band off the ground entirely and the planner scores viewpoints suspended
in air or buried in soil.

## 3. The planning cycle

The planner runs a state machine at 10 Hz. In exploration the cycle is:

**PLAN** — select the next viewpoint (§4–§7) and publish it as a goal.
**NAVIGATE** — monitor the drive; detect arrival, timeout or stall (§8).
**INTEGRATE** — hold still for a short settling window (2 s) so the mapper
folds in observations taken from the new pose before they are scored.
**LOG_STEP** — write one row of per-step metrics (§12) and return to PLAN.

Exploitation substitutes **EXPLOIT_PLAN** and **EXPLOIT_DWELL** for PLAN and
INTEGRATE, reusing NAVIGATE and LOG_STEP unchanged. Two further states,
**RETURN_NAV** and **RETURN_SYNC**, implement the multi-robot rendezvous
behaviour (§10.2), and **PROXIMITY_HOLD** implements the coordinated stop
(§10.3). A run ends in **DONE**, which either shuts the node down or leaves it
idling, ready to service targets released later.

A *step* is one full cycle, and the step count — not wall-clock time — is the
planner's own budget. Runs terminate on coverage saturation (§9), on the step
budget, or on the experiment's wall clock, whichever comes first.

## 4. Candidate generation

Each planning cycle the planner proposes a set of candidate viewpoints. A
candidate is a position and a heading, because what a sensor sees depends on
both.

**Local candidates** are drawn on a polar grid centred on the robot: three
distance rings (2, 5 and 8 m), eight bearings per ring, and four headings at
each position — 96 candidates in total. This grid gives dense, reliably
reachable options in the robot's immediate neighbourhood, and its regularity
means the planner always has something to choose from.

**Frontier candidates** supply the long range. A frontier voxel is a free voxel
adjacent to unobserved space — the boundary of what is known. Frontier voxels
anywhere in the region of interest are clustered into 5 m bins and one centroid
per bin is added as a candidate, oriented to face the unknown region. Without
these the planner is purely greedy and local: it will happily finish a room and
then have no candidate worth visiting, because every viewpoint within 8 m is
already fully known. Frontiers are what let it commit to crossing the map.

Candidates outside the region of interest are dropped, as are those falling in
occupied space when a map is available to check against.

## 5. Expected information gain

Each candidate is scored by simulating what the sensor would observe from it.
The planner casts a grid of rays matching the sensor's field of view — 16 by 12
rays over 60° horizontally and 45° vertically, from 0.3 m to 10 m — and walks
each ray through the map, accumulating a per-voxel score and stopping at the
first voxel confidently occupied (p ≥ 0.7), which simulates occlusion. Rays are
clipped to the region of interest before they are walked, so that space the map
does not cover contributes nothing rather than being scored as maximally
uncertain; without that clip the planner develops a systematic bias toward
staring over the boundary of its own map.

The per-voxel score is the **expected information gain**: the mutual information
between the voxel's unknown occupancy and the next observation of it,

```
EIG(v) = H(p) − E[H(y | θ)]
       = H(p) − [ ψ(s+1) − p·ψ(a+1) − (1−p)·ψ(b+1) ]
```

where `a = a_occ`, `b = a_free`, `s = a + b`, `p = a/s`, `H` is the binary
Shannon entropy and `ψ` is the digamma function. The first term is how uncertain
the *prediction* of the next measurement is; the second is how uncertain that
measurement remains once the true occupancy is known. Their difference is
therefore how much of the prediction's uncertainty an observation would actually
resolve — evidence gained, not noise re-measured.

The practical consequence is the behaviour a naive entropy measure gets wrong. A
voxel observed a hundred times as ambiguous has high entropy but near-zero
expected gain, because another look will not settle it. An unobserved voxel sits
at the `Beta(1,1)` prior and yields the maximum. The planner is thus pulled
toward genuinely new space rather than toward whatever is noisiest, which is the
failure mode of entropy-driven next-best-view planning in vegetation, where
partially transmissive foliage produces large volumes of permanently ambiguous
occupancy.

An alternative ray-marginalised formulation following Asgharivaskasi and
Atanasov (T-RO 2023) is implemented and available as an ablation: it weights
each voxel's contribution by the probability that the ray actually reaches it,
so occluded volume behind a wall stops inflating a viewpoint's score. A
trajectory-scoring variant, which sums the score over poses sampled along the
path to a candidate rather than at its endpoint alone, is likewise available and
off by default — it multiplies the cost of the scoring pass by the path length
and is the one setting that can push a planning cycle past its time budget.

## 6. Travel cost

Information alone would send the robot across the map for a marginally better
view. The planner therefore weighs each candidate against the cost of reaching
it.

When a 2D grid is available, the cost is the true path length: a single
8-connected Dijkstra flood outward from the robot's current cell, computed once
per planning cycle and then queried in constant time per candidate. The flood is
radius-bounded to just beyond the candidate ring, which keeps it under a
millisecond. It doubles as a reachability test — a candidate in a free pocket
sealed off by obstacles returns infinite cost and is rejected before a goal is
ever published, rather than after the navigator has spent a minute failing to
reach it.

When no 2D grid is published — the default and the field configuration — the
cost is straight-line distance and no reachability filtering is performed.
Unreachable goals are then caught after the fact by the navigation timeout and
blacklist (§8) instead of being screened out in advance. Exploitation uses the
same machinery but floods without a radius bound, because a tree target may be
released while the robot is on the far side of the map.

## 7. Selection

Candidates are ranked by information per unit distance:

```
U(c) = EIG(c) / (ε + cost(c)),    ε = 0.1 m
```

The form is deliberately parameter-free. There is no weight to tune between
information and distance: a candidate twice as far must be twice as informative
to win, and the constant ε only prevents division by zero for candidates at the
robot's feet. Unreachable candidates take `U = −∞` and sort last.

The planner then walks the ranking in order and takes the first candidate that
survives a series of filters: it must not be within arrival tolerance of the
robot's current position (a goal already reached wastes a step); it must lie in
free space and be reachable, when a map exists to check; it must not fall within
2 m of a goal that failed in the last 60 s (§8); and it must not be contested by
a teammate that has a better claim on it (§10.1). The surviving candidate is
published as a goal pose and broadcast as an intent.

## 8. Execution and failure recovery

The planner is optimistic about navigation and explicit about giving up.

Each drive gets a **distance-scaled time budget**, `clamp(d / v̂ × k, 30 s,
180 s)` with an assumed speed `v̂` of 0.15 m/s and a safety factor `k` of 3, so a
2 m hop is allowed 40 s and a 15 m crossing 180 s. The budgeted distance is the
*path* cost where one is available, not the straight line: a goal 5 m away that
requires a 15 m detour should not be failed for being slow.

Running in parallel is a **no-progress watchdog** that fails the goal
immediately if the robot has travelled less than 0.2 m in the last 15 s — the
wedged-robot case, which the time budget alone would take minutes to catch.
Travelled distance is integrated from per-tick pose deltas, with single-tick
jumps above 1 m rejected as localisation corrections rather than motion, so a
relocalisation cannot spoof the watchdog or inflate the logged path length.

Arrival requires both position and heading to be within tolerance (0.4 m,
0.4 rad), because a viewpoint's value depends on where the sensor points. The
in-place rotation after the position is reached gets its own separate 15 s
deadline; sharing the drive budget caused goals to be failed with the robot
already standing on them. The planner's tolerances must stay strictly looser
than the navigator's own goal checker, or the navigator stops inside its
tolerance but outside the planner's and the planner never registers arrival.

A goal that fails for any reason is **blacklisted**: a 2 m disc around it is
rejected for 60 s. This is what stops the planner from immediately re-selecting
the unreachable location it just failed at — without it, a goal behind an
unmapped obstacle is chosen again on the very next cycle, indefinitely. The
entry expires rather than persisting, so a location that becomes reachable as
the map grows is eventually retried.

## 9. Termination

Exploration ends when the region of interest is saturated: when the fraction of
unknown area falls below 5 % on three consecutive planning cycles. The
consecutive requirement guards against a single measurement gap ending a run
prematurely.

Coverage is measured as a **2.5D column statistic** over the fused 3D map — the
fraction of ground-plane columns inside the region of interest containing no
observed voxel at any height. This deliberately ignores volumetric unknowns.
Trunk interiors and canopy shadow are never observable and would put a permanent
floor under a true 3D unknown fraction, so a run would never terminate. What the
column measure asks instead is the operationally meaningful question: *has the
robot seen anything at all in this part of the area?*

The measure is sensitive to the region-of-interest box. Columns inside the box
that no robot can reach never clear, and put a floor under the unknown fraction;
the threshold must be calibrated above that floor for a given site.

## 10. Multi-robot operation

Each robot runs its own planner against its own copy of the fused map. There is
no central allocator and no leader. Coordination is achieved by three
independent mechanisms, each of which degrades to single-robot behaviour when
the robot is alone.

### 10.1 Goal deconfliction (MinPos)

Each robot broadcasts an **intent** at 1 Hz: its currently selected goal, its
own pose, a claimed disc radius (default: the sensor range, 10 m) and a
time-to-live (5 s). Peers keep the latest intent per robot and expire it when
the time-to-live lapses, so a robot that crashes or leaves radio range releases
its claims automatically.

When two robots want the same region, the tie is broken by the **MinPos** rule
(Bautin, Simonin and Charpillet, IROS 2012): the robot closer to the contested
viewpoint keeps it, and the other rejects the candidate and takes the next-best
one on its own ranking. Ties in distance are broken by comparing robot names
lexicographically. The comparison is total and antisymmetric, so exactly one
robot yields per conflict — never both, and never neither.

One implementation detail is load-bearing in the field: claim expiry is timed
from **local receipt**, never from the sender's timestamp. Robot clocks in the
field are not synchronised — offsets of hours have been observed in this
system's own logs — and an expiry computed from the sender's clock makes claims
either immortal or stillborn depending on the sign of the offset.

### 10.2 Rendezvous reconnection

A robot that loses radio contact keeps exploring alone; the difficulty is what
it should do when it *finishes*. Stopping is wrong, because its teammate may
still be exploring and their maps have not merged.

Instead, a robot that exhausts its goals while a teammate is out of contact
drives back to its **last-connected anchor** — the pose at which it last heard
from a teammate, which by construction lies inside radio coverage — and waits
there until the whole team is back in contact, broadcasting its presence
throughout so that arriving teammates can count it. It then re-plans against the
now-merged map: if the merge revealed new frontiers the team disperses again
(MinPos splits them), and if not, everyone reaches the end together.

The barrier therefore enforces a useful invariant: **a robot can only finish
when the whole team is present and the merged map is saturated**, so no robot
quits while a teammate is still working. The wait is unbounded by default, with
an optional timeout as a field escape hatch for a teammate that has genuinely
died. The barrier takes priority over exploitation: an open tree target is stood
down rather than serviced on the way home, because a robot detouring to inspect
trees would leave its teammate waiting indefinitely.

### 10.3 Coordinated proximity stop

MinPos deconflicts *goals*, not *paths*. Two robots holding different goals can
still be routed across each other — in one field trial the two commanded routes
crossed with zero closest approach.

While driving, each robot therefore watches its teammates' positions and yields
when a higher-priority teammate is moving nearby. Right of way is the
lexicographically smaller robot name — the same total order MinPos uses. The
key property is that the decision is computed **from identifiers alone**, with
no geometry: both robots of a pair always agree on who yields, even if their
estimates of each other's positions disagree, so the pair can neither deadlock
(both stopping) nor collide through mutual assumption (both proceeding).

The yielding robot cancels its in-flight navigation goal *and* publishes a
holding goal at its own current position. The redundancy is deliberate: the
cancel is only confirmed asynchronously, so the zero-travel goal covers a cancel
that is lost or rejected. It resumes the original goal once the peer has moved
beyond a release distance — 5 m to hold, 6 m to release, the gap providing
hysteresis so a peer skirting the threshold does not chatter the robot between
states.

Two release conditions prevent deadlock. A peer that has not moved for 10 s is
treated as **parked** and no longer held against, because a stationary robot is
an ordinary obstacle that the navigator's own costmap will route around, and
holding against one deadlocks — a teammate waiting at its rendezvous anchor
would otherwise stop the team forever. That release has a floor of 1.5 m,
though: a peer parked closer than that keeps the hold, since "it stopped" is no
licence to drive closer still. A final escape hatch resumes the drive after
120 s regardless. Held time is banked rather than refunded, so repeated holds
cannot grant one goal unbounded driving time.

The 5 m trigger is sized against the **reaction budget**, not the nominal
separation. Peer pose age, the planner's own tick, cancel propagation through
the navigator and physical braking together exceed a second, and only the
yielding robot slows — the pair keeps closing at the peer's speed throughout.
At field speeds an earlier 3 m threshold was consumed entirely by that latency.

This layer is best-effort **coordination, not a certified safety function**. It
requires live peer data, both planners running, and the navigator honouring the
cancel. In the field the crewed manual stop remains the hard backstop.

## 11. Perceptive exploitation

When a tree target arrives — a trunk centre, an estimated radius, and an
identifier — the planner preempts exploration, even mid-drive, and switches to
inspecting it. Targets arrive on a shared topic as a single message type; the
current experiments publish a preselected list on a timed schedule, but a live
detector can publish the identical message with no change to the planner. That
seam is why exploitation timing is an experimental variable rather than a
property of the perception stack.

**Vantage generation.** Three viewpoints are placed evenly around the trunk on a
standoff circle (120° apart), each facing the trunk axis. The standoff is the
trunk radius plus 2 m, clamped so the trunk surface stays beyond the sensor's
minimum range and the axis within its maximum. Ring positions are deterministic
functions of the trunk centre and radius, so index *k* names the same physical
pose on every robot — which is what makes cooperative capture possible (below).
The ring's starting angle is configurable and set to 30° in the field, because a
0° start put the first vantage of every ring straight down a plantation row,
which caused the other two to land inside neighbouring canopy and fail together.

**Validation.** Each vantage is re-checked every planning cycle, so an angle
blocked earlier can become available as the map improves. A vantage must lie in
the region of interest, on free and reachable ground, and — the
exploitation-specific test — must have a **clear line of sight** to the trunk: a
ray marched from the vantage toward the trunk axis at the vantage's own
measurement height must hit no confidently occupied voxel before reaching the
trunk surface. Voxels at or inside that surface are the trunk itself, an
expected hit rather than an occluder. Unobserved space never blocks, since a
sightline through unmapped volume is unknown, not known-bad.

**Execution.** The planner drives to the nearest valid, unvisited vantage and
**dwells** there for 8 s. No goal is re-sent during the dwell: a re-send is a
fresh navigation request, and the navigator emits at least one velocity command
even for an already-satisfied goal, which would put a rotation through the
middle of the capture the dwell exists to take. On completion the sightline is
re-checked from the pose the robot actually settled at, and it is that verdict —
not the one from selection time — that is credited and logged.

**Completion.** A target succeeds once three clear-sightline dwells are
credited, and closes as *partial* if the per-target budget (300 s since the last
progress event) expires first. The budget is what guarantees that one bad tree
cannot consume the mission; progress events — activation, arrival at an approach
waypoint, a completed dwell — re-arm it, and proximity-hold time is refunded so
yielding to a teammate never costs a target.

**Approach fallback.** If no vantage is currently selectable — the ring is
unmapped, occluded or unreachable — the planner does not abandon the target. It
drives to the nearest reachable point on the line toward the trunk, mapping the
area en route, and re-plans the ring on arrival. This converts "the ring is not
visible from here" into an information-gathering action rather than a failure,
and termination is preserved because each approach must land meaningfully closer
than the last.

**Cooperative capture.** When two robots work the same trunk, each broadcasts a
bitmask of the ring indices it has captured with a clear sightline. Peers merge
the union into their own record of that target, so the three-vantage quota is a
**team** quota: the ring is covered once, cooperatively, and the trunk closes
for everyone. Because indices name canonical poses and the mask is cumulative
state rather than an event, merges are idempotent and a lost message loses no
credit. A separate, much smaller MinPos disc is used for vantage claims — the
exploration-scale disc would swallow the entire ring and cause one robot to veto
the trunk rather than the two of them splitting its angles.

## 12. Coupling to the mapper

Exploitation also drives map resolution. When a target becomes active the
planner registers a **refinement region** — a cylinder around the trunk — with
its own mapping node, and unregisters it when the target closes. Inside that
cylinder the mapper integrates at a finer voxel resolution than the surrounding
survey map. The result is that fine detail is spent only where inspection
actually happens, keeping the global map affordable to store and to transmit
between robots. Regions belonging to targets that never completed are
unregistered on shutdown, so a run cannot leave the mapper refining trunks
indefinitely.

## 13. Instrumentation

One CSV row is written per step, carrying: the step index and timestamp; total
observed voxels, frontier count and mean information/entropy/variance over the
map; distance travelled; the selected candidate's information gain, path cost
and final utility, alongside the means over all candidates that cycle, so a
selection can be attributed post hoc to either term; planning time; the number
of active peers and the counts of candidates rejected by deconfliction and by
unreachability; cumulative proximity-hold count and held seconds; and, for
exploitation rows, the target, the vantage index, how many vantages were valid,
whether the sightline was clear, and the dwell time achieved.

The columns are chosen so that behaviour is reconstructable without the logs:
whether a robot was pulled off course by a teammate, whether a trunk closed
because it was captured or because it timed out, and whether a viewpoint was
selected for its information or merely for its proximity.

## 14. Assumptions and limitations

The planner assumes a **ground robot**: all distances, deconfliction discs and
sightline tests are computed in the horizontal plane, with height handled by
snapping viewpoints to the local ground surface rather than by planning in three
dimensions. The vantage ring is likewise a single horizontal circle; there is no
height-varying inspection pattern.

The region of interest is an **axis-aligned box**. A survey area at an angle to
the map frame must be entered as its bounding box, which admits candidates
outside the true area and, because the same box defines the coverage measure,
puts a floor under the unknown fraction that the termination threshold has to be
calibrated above.

Obstacle avoidance is **delegated**. In the default configuration the planner
performs no 2D reachability filtering at all; it discovers that a goal was
unreachable only when the navigation budget expires, and recovers by
blacklisting rather than by prediction.

The proximity stop is **coordination, not safety**. It depends on live peer
data, on both planners running, and on the navigator honouring a cancel. In
simulation runs to date the cancel path was never exercised — the simulated
navigator exposes no cancellable action interface, so every cancel failed and
the redundant holding goal was the sole mechanism that stopped the robot. That
redundancy is why the behaviour worked at all in those runs, but it means the
cancel path itself remains untested outside the field stack.

Finally, the planner **positions and dwells; it does not reconstruct**. Whether
the captured views are sufficient for the downstream measurement is decided
offline, and the planner's own success criterion — three clear-sightline dwells
— is a geometric proxy for that, not a measurement of it.

---

### Key parameters

| Parameter | Default | Role |
| --- | --- | --- |
| Candidate rings / bearings / headings | 3 / 8 / 4 | 96 local candidates per cycle |
| Candidate radii | 2–8 m | Local viewpoint spacing |
| Frontier cluster size | 5 m | Long-range candidate granularity |
| Sensor model | 60° × 45°, 0.3–10 m, 16 × 12 rays | Simulated field of view for scoring |
| Occlusion threshold | p ≥ 0.7 | Ray stops at confidently occupied voxels |
| Utility ε | 0.1 m | Division guard in `EIG / (ε + cost)` |
| Nav budget | `clamp(d/0.15 × 3, 30, 180)` s | Distance-scaled drive timeout |
| No-progress watchdog | 0.2 m in 15 s | Wedged-robot detection |
| Arrival tolerance | 0.4 m, 0.4 rad | Must exceed the navigator's own checker |
| Failed-goal blacklist | 2 m for 60 s | Prevents re-selecting failed goals |
| Coverage termination | < 5 % unknown, 3 consecutive cycles | Saturation criterion |
| MinPos claim | 10 m disc, 5 s TTL, 1 Hz | Goal deconfliction |
| Proximity hold / resume | 5 m / 6 m | Coordinated yield with hysteresis |
| Parked release / floor | 10 s unmoved / 1.5 m | Deadlock avoidance and its limit |
| Vantages per trunk | 3 at 120°, 30° start | Prescribed angular coverage |
| Vantage standoff | trunk radius + 2 m | Clamped to the sensor envelope |
| Dwell time | 8 s | Capture window per vantage |
| Per-target budget | 300 s since last progress | Bounds time spent on one trunk |
