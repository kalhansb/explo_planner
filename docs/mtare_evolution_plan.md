# Implementation plan: evolving explo_planner into an M-TARE-style distributed explorer

*Status: v4.1. P0–P7 implemented; no validation campaign run yet.
Implementation lives on `explo_path_planner_experiments` (NOT the
`mtare_evolution` branch named in v1 — that branch was never cut, and this
document is committed rather than held). Reference clone:
`mtare_planner_ref/` at the workspace root (caochao39/mtare_planner, branch
`noetic`, COLCON_IGNOREd).*

---

## 1. Goal

Evolve `explo_planner` from a flat, per-robot frontier/EIG planner with
*reactive* pairwise reconnection into a simplified M-TARE-style distributed
explorer: a hierarchical planner in which robots share a coarse cell-level
world model, divide exploration work by each solving the same allocation
problem and executing only their own route, and treat reconnection
(rendezvous, pursuit, hybrid) as a *planned, utility-gated* activity rather
than a timer-triggered reaction.

We port **concepts, not code** (mTARE is ROS1/catkin; we are ROS2). The
portable essence of mTARE is small — roughly 3k LOC of ideas out of its 26k:

| mTARE concept | mTARE home | Size |
| --- | --- | --- |
| Coarse cell world + status machine + per-cell sync bookkeeping | `grid_world.cpp` (slice, incl. `update_id`/reset rules) | ~1.5k |
| Comms manager (liveness+range+handshake+closure) | `multi_robot_exploration_manager.*` | 773 |
| Robot/plan structs + wire codec | `robot.*` | 269 |
| Scheduled rendezvous (as *inspiration* — see §3.5) | `rendezvous_manager.*` | 267 |
| Pursuit MDP (peer position prediction) | `pursuit_mdp.*` | 181 |
| Per-cell knowledge bitmask | `knowledge_base.*` | 92 |

Note the knowledge port is *not* just the 92-line bitmask file: the
load-bearing parts are the reset-on-status-change and guard-table semantics
that live in `grid_world.cpp` and `multi_robot_exploration_manager.cpp`
(§3.2). Budget it as part of the ~1.5k grid-world slice.

What we explicitly do **not** port: OR-Tools VRP (replaced by a deterministic
greedy allocator), the TOPwTVR/PBS/CBS pursuit solver (~2.6k LOC, replaced by
MDP-argmax interception), the keypose graph (replaced by a cell adjacency
graph), rolling grids/planning env (MapCache already covers this), convoy
mode, heterogeneous mobility types, the `kSyncDelay` delta-sync protocol,
and the ros1_bridge topic map (DDS does this natively).

## 2. What already exists, and what maps to what

explo_planner already implements — validated in the field and across the sim
campaign generations — several things mTARE also has, in different form:

| Capability | explo_planner today | mTARE | Plan |
| --- | --- | --- | --- |
| Local viewpoint planning | EIG candidates + `U = EIG/(ε+cost)^γ` in `doPlan` | ViewPointManager + local TSP | **Keep ours** as the local layer, scoped by the global layer |
| Goal deconfliction | MinPos intents (`coordination.*`) | implicit via shared VRP | Keep as backstop under the allocator |
| Comms sensing | link-gate (`link_states.csv` emulator) + intent TTL | range+liveness+handshake+closure | Generalise to N robots, add handshake + closure |
| Rendezvous | reactive: return to midpoint of last-contact pair | scheduled: agreed (cell, time), proposal/echo protocol | Appointment `(cell, t_meet)` derived from the allocator's tours — agreement by construction, no protocol (§3.5); midpoint survives as the floor |
| Pursuit | chase trail = [peer_goal, peer_pose], staleness budget | MDP prediction over peer's shared route + TOPwTVR | Keep chase skeleton; upgrade target via MDP over shared tour |
| Hybrid | pursuit budget, then midpoint of last-contact pair | relay plan accepted iff strictly better than no-comms baseline | Add the utility gate; midpoint fallback unchanged |
| Global structure | none (flat) | GridWorld cells + VRP tours | **The core new work** |
| Inter-robot state | `RobotIntent` only (goal, claim, presence, map-size beacon) | `ExplorationInfo` (cells, roadmap, positions, plans) | New `TeamWorld` message |
| Map sharing | out-of-process (dscovox `scovox_bin` deltas) | shared coarse cell map | Unchanged for voxels; the *cell* layer is what the planner shares |
| Team size | hardwired 2 (link gate refuses N≠2) | N robots | Generalise data structures now, harness later |

Key seams the current code already provides (this is why the refactor is
tractable without touching the 9.3k-line node monolith's structure):

- Reconnection policy is value-typed and pure in `planner_util.hpp`
  (`ReconnectMode`, `pursuitBudgetSec`, `meetingPoint`, `teamComplete`).
- Peer belief is one object with a clean API (`coordination.hpp`).
- All state transitions funnel through `transitionTo` (one insertion point).
- Nav is one publisher (`publishGoal`) + one canceller (`abandonNavGoal`).
- Telemetry is schema-versioned NDJSON (`experiment_log.hpp`, v3) + an
  append-only 32-column CSV.

## 3. Target architecture

Five new library units (each pure, each with a gtest suite mirroring the
existing 15), two new messages, and thin node glue:

```
                        ┌─────────────────────────────┐
 scovox fused map ────► │ MapCache (existing)         │
                        └──────────┬──────────────────┘
                                   ▼
                        ┌─────────────────────────────┐     TeamWorld msg
                        │ cell_world  (NEW, pure)     │ ◄──── from peers
                        │ 2D coarse cells over ROI    │ ────► to peers
                        │ status machine + known_by   │
                        └──────────┬──────────────────┘
                                   ▼
 link gate /            ┌─────────────────────────────┐
 intent TTL ──────────► │ team_model  (NEW, pure)     │
                        │ N-robot comms status,       │
                        │ closure, lost-peer test     │
                        └──────────┬──────────────────┘
                                   ▼
                        ┌─────────────────────────────┐
                        │ global_allocator (NEW,pure) │
                        │ greedy makespan tours,      │
                        │ no-comms / assume-comms     │
                        └───────┬───────────┬─────────┘
                          focus cell   reconnect value
                                ▼           ▼
                 ┌──────────────────┐  ┌──────────────────────────┐
                 │ doPlan (existing)│  │ reconnection (existing   │
                 │ candidates       │  │ state machine) + NEW:    │
                 │ filtered to      │  │ rendezvous_scheduler,    │
                 │ focus cell       │  │ pursuit_predictor        │
                 └──────────────────┘  └──────────────────────────┘
```

### 3.0 Fleet identity (prerequisite)

Every bitmask, tie-break, and adoption rule below needs a stable numeric
robot id agreed by the whole fleet — which does not exist today: identity is
a free-form string (`RobotIntent.robot_id`), and the only numeric index in
the codebase is the link-gate robot-index topic, which is COMMS=1-only,
N=2-only, and allowed to be unusable at runtime. So: a required ordered
string-array param `team_robot_names`; a robot's numeric id is its position
in that array. `TeamWorld` echoes the array's hash, and a mismatch with a
peer is a loud, fatal config error. Declared in P0, independent of the comms
emulator.

### 3.1 `cell_world` — the coarse global map

A 2D grid of cells over the ROI in the map frame. mTARE's grid world is
genuinely 3D as shipped; we flatten it deliberately, matching the
ground-robot assumption already stated in planner_method.md §14.

- `cell_size_m` param, default 10.0 (mTARE's derived cell size in its
  *forest* config — the relevant domain — is 9.6 m).
- Per-cell state computed from MapCache column statistics (the same 2.5D
  column machinery `coverageUnknownFraction` already uses): unknown-column
  fraction, frontier-voxel count, observed-voxel count.
- Status enum ported: `UNSEEN, EXPLORING, COVERED, COVERED_BY_OTHERS,
  EXPLORING_BY_OTHERS` (NOGO deferred). Transitions use hysteresis
  thresholds like mTARE's `kCellExploringToCoveredThr /
  kCellCoveredToExploringThr` pair to stop status flapping.
- Per-cell `update_id`: a **local** counter, incremented only on
  self-observed committed status changes (mTARE's rule — the merge path
  never increments it). Cross-robot `update_id` comparison is therefore
  *not* an ordering of information quality and is never used as one (§3.2).
- Per-cell `known_by` uint32 bitmask with mTARE's **reset semantics**: every
  committed status change resets `known_by` to {self}, because the change
  invalidates what peers knew. Without the reset, the §3.6 knowledge gate
  goes vacuous for any cell a peer ever heard of (it would conclude
  "nothing to tell them" about a cell whose status changed after
  separation), and the lost-peer test misclassifies. OR-merging happens
  only when statuses agree (§3.2).
- A **cell adjacency graph** replaces mTARE's keypose graph + roadmap:
  8-connected cell centroids, edge weight = centroid distance, an edge
  disabled when the plan-map occupancy fraction along it exceeds a threshold
  (when no plan map: all edges enabled, matching today's straight-line
  fallback philosophy). All-pairs shortest paths on ≤ a few hundred nodes is
  low single-digit milliseconds per cycle (Floyd–Warshall at 400 nodes;
  Dijkstra-per-source if that ever shows up in `plan_time_ms`).

#### 3.1.1 Threshold calibration (measured during P1, not assumed)

The first P1 build shipped the thresholds this section implies by analogy
with mTARE: `covered_max_unknown = 0.15`, `exploring_min_unknown = 0.35`,
and a frontier veto expressed as an absolute voxel count (`≤ 4`). A full
1800 s two-robot run, reaching the harness's own completion criterion
(`u_roi = 0.634` against `done_unknown_fraction = 0.64`), promoted
**exactly zero cells**. Every cell in both robots' censuses sat at
EXPLORING from start to finish.

An all-EXPLORING census is what you get from a broken census *and* from a
threshold nothing can satisfy, and nothing in the census row could
distinguish them. So the row was widened rather than the thresholds
guessed at: `cell_census` now also carries `cells_measured`,
`cells_frontier_ok`, and order statistics of both per-cell measures over
the measured cells (`cell_unknown_{min,p10,median}`,
`cell_frontier_frac_{min,median}`, and the joint reading
`cell_frontier_frac_at_best_unknown`). The two marginals alone cannot
answer whether promotion is *possible*, because the best-unknown cell and
the best-frontier cell need not be the same cell; the joint field asks that
question directly.

Measured on a 900 s run, stable across both robots and flat from
t ≈ 300 s onward:

| quantity | value |
|---|---|
| `cell_unknown_min` (best cell on the map) | 0.360 |
| `cell_unknown_p10` | 0.422 |
| `cell_unknown_median` | 0.496 |
| ROI-wide `coverageUnknownFraction` | 0.637 |
| `cell_frontier_frac_min` | 0.333 |
| `cell_frontier_frac_median` | 0.881 |
| `cell_frontier_frac_at_best_unknown` | 0.833 |

The best-observed cell never gets below 0.360 unknown, so `0.15` was
unreachable by construction — there was no defect to fix.

The first response was to re-scale the library defaults to that
distribution: promote at the p10 (0.42), release at the median (0.50). A
fresh run then scored `sim/gate_p1.py` and **failed it again**, with the
best cell bottoming out at 0.4232 against the 0.42 threshold. That second
run was shorter (makespan 332 s against the calibration run's 495 s and
still going), so its cells had less dwell and the floor sat higher. A
threshold fitted to one run's tail did not survive the next run.

That failure is the useful one, because it identifies the real category
error. **These are world-calibrated knobs, not constants.** The codebase
already contains the precedent, and it is exactly parallel:
`done_unknown_fraction` ships at `0.05` in `shared_params.yaml` and the sim
harness overrides it to **0.64**, because — in that file's own words —
"never-visited out-of-AO columns put a permanent floor under the unknown
fraction: calibrate `done_unknown_fraction` above that floor". The per-cell
measure counts the same thing over a smaller footprint and inherits the
same floor for the same reason. Pinning a library default to one world's
floor was the mistake; the fix is to put the calibration where the world is
described.

So:

- **Library defaults stay with the saturating-map world model**
  (`covered_max_unknown = 0.15`, `exploring_min_unknown = 0.35`,
  `covered_max_frontier_frac = 0.90`), consistent with the shipped
  `done_unknown_fraction: 0.05`. Wrong elsewhere, but *loudly* wrong: a
  floor above the threshold pins every cell at EXPLORING.
- **The sim harness calibrates them**, next to its `DONE_UNKNOWN=0.64`:
  `CELL_COVERED_U=0.55`, `CELL_EXPLORING_U=0.62`, `CELL_FRONTIER_FRAC=0.95`,
  all recorded in `run_manifest.txt` because a census is not comparable
  across two runs that disagree about what COVERED means.

The sim values are placed by the same rule as `DONE_UNKNOWN`: release just
under the level the mission itself accepts for the whole ROI (0.64), since
a cell that has degraded to the mission's own accept level is not
distinguishably explored, and promote a band's width below that. The band
has to clear the run-to-run spread of the floor (0.36–0.43 observed), not
one run's point estimate — which is the specific thing 0.42 failed to do.

The frontier veto changed **type**, not just value. An absolute voxel count
is not a quantity that can be set correctly: it means something different
at every cell size and map resolution, and at 10 m cells with 0.1 m voxels
a thoroughly swept cell holds tens of thousands of voxels and hundreds of
boundary ones. `covered_max_frontier_frac` is the same measure with the
scale divided out, so one number means the same thing at any cell size.
`frontierFraction()` reports "cannot measure" as `-1.0`; note that a
negative sentinel passes an upper-bound comparison, so `classify()` tests
for a real measurement rather than trusting that it got one. How much the
veto discriminates is itself world-dependent — a frontier voxel is one with
an absent 6-neighbour, so a densely ray-traced map isolates the true
boundary while this sparse surface-shell map puts 79–89% of every cell's
voxels in that category. A threshold inside that band splits the population
near its own median and reports mostly noise, which is why the sim raises it
to 0.95 and lets the column measure do the discriminating.

Three disciplines this imposes on anyone re-tuning these:

- **Re-read, do not re-guess.** The quantiles are on every census row. Take
  them from a fresh run of the build in question.
- **Do not fit and certify on the same run.** The 900 s run above chose the
  first set of thresholds and was therefore disqualified from scoring
  `sim/gate_p1.py` — which is the only reason the 0.42 failure was caught
  before the number reached P3.
- **Fit to the spread, not to a point.** This sim is nondeterministic
  run-to-run; a threshold set from one run's plateau is a threshold set from
  one draw of it.

### 3.2 `TeamWorld` + `CellState` messages (explo_planner_msgs)

Two new messages — *not* an extension of `RobotIntent` (its own comment
block warns that changing it silently drops mixed-build producers on Humble;
a new type fails loudly instead, which is the failure mode we want).

```
# CellState.msg
uint16  id
uint8   status            # normalised: UNSEEN / EXPLORING / COVERED only
uint32  known_by
uint32  update_id

# TeamWorld.msg
Header    header
uint8     robot_id               # index into team_robot_names
uint32    team_hash              # fleet-identity check (§3.0)
geometry_msgs/Point position
uint32    in_range_mask          # peers this sender currently hears directly
CellState[] cells
uint16[]  my_tour                # cell ids of sender's current global route
bool      finished               # exploration latched done on sender's map
int32     rendezvous_cell_id     # -1 = no proposal
float32   rendezvous_time_sec    # mission-elapsed seconds (§3.5)
geometry_msgs/Point[] robot_positions   # gossip: last known position per robot
float32[] robot_last_heard_sec          # gossip: mission-elapsed last direct contact per robot
```

- **Wire normalisation** (mTARE's rule, and load-bearing): statuses are
  normalised on send — `COVERED_BY_OTHERS` ships as `COVERED`,
  `EXPLORING_BY_OTHERS` ships as `EXPLORING`. The `*_BY_OTHERS` values are
  local bookkeeping and never appear on the wire; without this, relayed
  state at N=3 is uninterpretable and the solve-same premise breaks.
- **Position/last-heard gossip**: mTARE relays third-party positions and
  update times through the middle robot; we keep that (it is what makes
  closure useful at N=3). Tours are *not* gossiped — only the sender's own
  (§3.7 degrades accordingly). Sizes are fixed at team size, a few dozen
  bytes.
- Published at `team_world_hz` on `exploration/team_world`, routed through
  the comms emulator like intents. **Publishing is off by default**
  (`team_world_hz: 0.0`); the harness turns it on per arm.
- **Full-state, not deltas.** At ≤400 cells × 11 bytes this is a few KB —
  affordable even at 1 Hz. Full state makes merge idempotent and loss-free
  by construction.
- **Merge rule** on receipt (in `cell_world`), a port of mTARE's guard
  table rather than a fresh design — "higher update_id wins" alone is
  wrong, because update_ids count per-robot events and an out-of-comms
  robot legitimately re-commits EXPLORING at a high count against a peer's
  first-hand COVERED:
  - peer `EXPLORING` applies only to local `UNSEEN` or
    `COVERED_BY_OTHERS` (→ `EXPLORING_BY_OTHERS`);
  - peer `COVERED` applies to local `UNSEEN`, `EXPLORING`, or
    `EXPLORING_BY_OTHERS` (→ `COVERED_BY_OTHERS`);
  - peer updates never downgrade a local first-hand `COVERED`, and are
    refused entirely for cells in the robot's own neighbourhood currently
    held `EXPLORING` (mTARE's local-priority rule);
  - `known_by` is ORed **only between matching statuses**; on adopting a
    peer status, `known_by` resets to {self, sender} (mTARE's
    merge-side reset);
  - all remaining conflicts resolve toward the more-explored status
    (`COVERED > EXPLORING > UNSEEN`) regardless of update_id, so a lost or
    reordered message can delay but not regress the census.
- **Executor budget**: the claim-grace episode proved the single-threaded
  executor can starve subscriptions during heavy planning, so the callback
  only copies the message; merging happens in the planning tick and is
  O(cells).
- **Plumbing this costs real work outside this repo** (budgeted in P2): the
  comms emulator relays only topics listed in its config, so
  `hmr_sim/config/comms_sim_params.yaml` gains the topic (cross-repo
  change); the planner grows split pub/sub params mirroring
  `coord_intent_pub_topic` / `coord_intent_sub_topics`; the harness wires
  the rx/ streams, bag topics, and readiness gates.

### 3.3 `team_model` — N-robot comms status

Port of the decision core of `multi_robot_exploration_manager`:

- Per-peer comms state from: intent/TeamWorld liveness (existing TTL
  machinery), the link gate where available, and the **mutual handshake**
  (peer's `in_range_mask` must include self) — this catches the one-way
  contact failure documented in `doc/limitations.md` §10.
- **Transitive closure**: peer B is `IN_COMMS` if any in-comms peer hears B
  directly. With N=2 this degenerates to today's behaviour exactly.
- Closure is a *comms-status* statement only. Every consumer that needs
  peer **data** (allocator vehicle starts, gates, scheduler agreement)
  keys on that peer's per-robot *freshness* — gossiped
  `robot_last_heard_sec` and position — never on `IN_COMMS` alone. At N=3
  in an A—B—C chain, A marks C in-comms (so no reconnection is dispatched
  at C) while still planning with C's gossiped-stale position, which is
  the correct behaviour and matches mTARE's design.
- `CommsStatus` per peer: `IN_COMMS / LOST_COMMS` (mTARE's RELAY_COMMS and
  CONVOY states are not ported).
- Lost-peer test (port of `CheckLostRobot`): a peer is `lost` when no cell
  it is known to know about is still worth intercepting at — this depends
  on the §3.1 reset semantics being in place to mean anything.
- Replaces the N=2 refusal in the robot-index parser with an N≤8 general
  path (uint32 masks; 8 is a policy cap, not a representation cap).

### 3.4 `global_allocator` — solve-same-take-own, without OR-Tools

The load-bearing mTARE idea: **no negotiation, no auction, no leader**. Every
robot merges what it has heard, solves the *same* allocation problem, and
executes only its own route. Divergent world views produce divergent
solutions; that is tolerated because everyone re-solves every cycle, and the
existing MinPos goal deconfliction plus goal blacklists remain underneath as
the collision backstop. Worst case is duplicated cell work — the redundancy
the campaign already measures.

- **Input set**: all cells with status `EXPLORING` **or**
  `EXPLORING_BY_OTHERS` (mTARE includes both; restricting to first-hand
  EXPLORING would make the two robots solve provably different problems
  after every exchange, destroying the solve-same premise).
- **Vehicle set**: all live, *unfinished* robots at their freshest known
  positions (gossip included). A peer whose `finished` flag is set is
  removed from the vehicle set and cells assigned to it return to the pool;
  the §3.6 knowledge gate likewise ignores finished peers.
- Algorithm (deterministic, replaces the 80 ms OR-Tools VRP): greedy
  makespan-balanced insertion. Repeatedly take the unassigned cell whose
  best insertion increases the maximum route cost least; ties broken by
  (cell id, robot id) so all robots break ties identically; costs
  integer-quantised (mm on the cell graph) before comparison so float
  associativity cannot break cross-robot determinism. Optional capped 2-opt
  polish. Cost = cell-graph shortest path (§3.1).
- **Comms-masked variants**, the port of mTARE's core comms-aware cost term:
  - `no_comms` plan: an out-of-comms robot may only be assigned cells whose
    `known_by` includes it — the port of the INF-masking in
    `GetDistanceMatricesNoComms`.
  - `assume_comms` plan: masks lifted for a hypothesised set of reconnected
    robots.
  - The **reconnection value** is the cost gap between them (§3.6).
- **Hierarchy coupling**: the first cell of my tour is the **focus cell**.
  `doPlan` keeps its exact candidate/EIG/utility machinery but restricts
  frontier candidates to the focus cell **plus its 8 neighbours** — the
  neighbourhood matters because frontier candidates are cluster
  *centroids*, and a cluster straddling a cell boundary parks its centroid
  in the adjacent cell; filtering to the focus cell alone would starve
  exactly the cells the clusters span. Local polar-ring candidates stay
  unrestricted. If the focus neighbourhood yields no admissible candidate,
  advance to the next tour cell; if the tour is empty, fall back to
  today's unrestricted behaviour (which is also the allocator-off arm).
- **Staleness rule**: a cell skipped k consecutive times (param, default 3)
  without producing an admissible candidate is force-recomputed from
  MapCache and, if still admissible-candidate-free, demoted to `COVERED`
  (the analogue of mTARE's not-connected → COVERED demotion). This stops a
  phantom EXPLORING cell from being re-assigned forever and from polluting
  the rendezvous objective, which ranges over the solved tours (§3.5).

### 3.5 `rendezvous_scheduler` — pre-planned meetings from the grid world

**A redesign, not a port.** Since v4 the *objective* differs from mTARE's
as well as the protocol; §3.5.1 records the audit that forced both changes.

The governing idea: **a meeting is not a detour to a landmark, it is a
constraint on the tours the robots were already driving.** Two robots
exploring a shared ROI pass near each other repeatedly; the rendezvous is
the cheapest of those space-time near-misses, not a third place both must
pay to reach.

- **What is agreed**: a `(cell, t_meet)` pair.
- **Cell.** Candidates are the cells already on some robot's tour,
  `∪ᵢ tours[i]`, plus the last-contact midpoint as a guaranteed-available
  floor (§3.6). Score each by the makespan penalty of forcing *every*
  robot's tour through it:

      penalty(c) = makespan(tours, each with c inserted at its best position)
                 − makespan(tours as solved)

  and take the argmin. For the robot whose tour already contains `c` the
  detour is zero; the other pays only an insertion. Both terms are
  `routeCost` and the makespan loop §3.4 already computes — no new cost
  model. **This replaces the v2/v3 minimax 1-centre**, which optimised
  distance *between frontier cells*: a quantity no robot pays, blind to
  where the robots are, where they are going, and what the trip costs.
  mTARE's own minimax has the same defect and worse — it offsets past the
  robot rows of its distance matrix (§3.5.1), so robot positions are
  excluded from the objective by construction and nothing bounds the
  meeting point's distance from the team.
- **Time.** `t_meet = maxᵢ τᵢ(c)` — the later robot's arrival under its own
  tour. **Not a formula**: no `min_interval + farthest/(2·v̂)`, no clamps to
  tune, because the tours already determine it. Capped by the
  map-divergence model already in the node (`rate_sum`,
  `explo_planner_node.cpp:6334-6340`, the summed map-growth rate of both
  robots): meet sooner when the maps are diverging fast enough that the
  exchange is worth more than the tours alone imply. That model is already
  symmetric in the two robots, so both cap identically.
- **Agreement needs no protocol.** §3.4 already guarantees tours are
  bit-identical *across processes* (integer-mm quantisation, total
  tie-break order, no clock/random/address-derived value). A rendezvous
  derived purely from the allocation output is therefore bit-identical too.
  There is no proposal, no echo, no lowest-id adoption, no freeze/chatter
  control, and no convergence bound to unit-test — agreement is a
  consequence of identical arithmetic over a shared world, exactly the
  argument `global_allocator.hpp` already makes for allocation itself.
  **This deletes the entire v2/v3 agreement protocol**, which was the bulk
  of P5's cost, and removes the flapping failure mode the chatter control
  existed to suppress.
- **Clock.** The countdown runs on **mission-elapsed time**, never absolute
  stamps: field clocks drift hours apart (the 2026-07-06 bunker/curt bags
  were 4531 s apart while recording simultaneously). Robots start their
  mission together, so elapsed-since-start is the only clock both ends can
  trust without synchronising. Same reasoning as claim-TTL-from-local-receipt.
- **Freeze at contact loss.** After separation the worlds diverge, so the
  tours diverge, so a re-derived rendezvous would differ between robots.
  Each robot freezes the pair derived from the last **bidirectionally
  confirmed** TeamWorld snapshot (§3.4's handshake). Bidirectional is
  load-bearing: under one-way loss the two ends otherwise disagree about
  when contact ended and freeze different pairs. This extends the snapshot
  discipline already at `explo_planner_node.cpp:6354-6360`, where the
  reconnect pair is frozen for exactly the same reason.
- **Execution reuses existing machinery unchanged**: `startReturnTo →
  RETURN_NAV → RETURN_SYNC` with the cell centroid as destination; arrival
  tolerance, wait caps, hold escalation and the release-on-contact
  invariant all as-is. Departure is when

      t_meet − mission_elapsed  ≤  travel_time(me → cell) · safety + margin

  which is **per-robot**: departures stagger, arrivals coincide. The robot
  further from the cell leaves earlier. This is precisely the leave-early
  rule mTARE computes and then discards (§3.5.1, edge 1).
- **Candidate admissibility**: only cells present in every live peer's
  `cell_world` and reachable on the proposer's roadmap are eligible. mTARE
  omits this check and `exit(1)`s on its absence (§3.5.1, edge 4).
- **No-show**: the existing wait-cap ladder, then resume exploring with the
  next schedule armed. A missed meeting degrades to "try again later",
  never to a standstill — mTARE deadlocks permanently here (§3.5.1, edge 5).

#### 3.5.1 What the reference actually does (audit, 2026-08-31)

Audited against `mtare_planner_ref/` @ `57b0a18` (the repo's only commit,
"initial commit", 2024-01-08; squashed import, no upstream history). This
is a **vendored reference copy, not our planner**. Recorded because several
of our design choices are reactions to specific defects, and a future
reader must be able to check that the reactions are still warranted.

**Provenance first: rendezvous is mTARE's baseline, not its method.** The
repo's `README.md` documents the pursuit/relay strategy as the paper's
contribution and never mentions rendezvous. `kRendezvous` defaults false
and is not parameter-controlled: `SetCommsConfig()`
(`grid_world.cpp:5004-5089`) unconditionally overwrites it from `kTestID`,
so `coordination.yaml`'s rendezvous knobs are inert. `kRendezvousTimeInterval`
is read and **never used** (its two defaults, 10 and 20, disagree with each
other and with the real hardcoded 120/300). `kRendezvousType` is dead, so
the "middle/nearest/farthest" variants are the same algorithm — which is
why our own `nearest`/`farthest` ablation params were dropped in v4 rather
than ported. **There are zero tests** on any of it: `CMakeLists.txt:195` is
a commented-out `# # Testing ##` banner with no targets. We are not
adopting a proven rival design; we are taking an idea from an untested
comparison arm.

Defects that our spec above is written against:

1. **The travel-time departure test is commented out.**
   `rendezvous_manager.cpp:141-142` keeps `if (remaining_exploration_time < 0)`
   and comments out `if (time_to_rendezvous >= remaining_exploration_time)`.
   `time_to_rendezvous` is the function's only parameter and is never read —
   a dead argument. Its producer is in a different file
   (`grid_world.cpp:4838`, `path_length / 2.0`, an implicit 2.0 m/s
   hardcoded rather than read from `kRobotSpeed`). Consequence: robots
   depart *at* zero rather than early, so each is late by its own travel
   time and **the lateness is heterogeneous** — a far robot arrives much
   later than a near one. Partial compensation exists (the interval folds
   in `distance_to_farthest_cell / 2`) but the ordering is still wrong.
   Our departure rule above is the restored form of the deleted test.
2. **Adoption is hardcoded to robot 0**, not lowest-id:
   `if (robots[i].in_comms_ && i == 0) // tmp fix: only get from robot 0`,
   with the lower-id rule commented out above it. Leader-follower in
   practice. Only `SyncNextRendezvous` retains `i < self_id`, and it
   ascends and breaks on first match, so it adopts the **lowest**-id
   in-comms peer — determinism a convergence argument would need, and which
   "any peer" would not give. We avoid the whole question: agreement by
   identical arithmetic (above) has no adoption step.
3. **Agreement is exact integer equality on both fields**, no tolerance and
   no versioning, over `ExplorationInfo.global_cell_ids` — a field named
   for something else, with the pair framed by `-2`/`-1` sentinels
   (`grid_world.cpp:3120-3127`; the trailing `-1` is load-bearing, without
   it `DecodeToOrderedCellIDs` never emits the row). Cell ids and intervals
   share an integer namespace with the sentinels, unescaped. In rendezvous
   mode the real relay-comms plan is not transmitted at all — the channel
   is fully repurposed.
4. **Unreachable rendezvous cell is an `exit(1)`.** `PlanForRendezvous`
   guards only the *from* node (`grid_world.cpp:4832-4836`); inside,
   `Graph::GetShortestPath` asserts `HasNode(to_node_id)` (`graph.h:478`)
   and `MY_ASSERT` is `exit(1)` (`misc_utils.h:39-45`), live in release.
   Reachable in practice: the cell may be adopted from another robot's
   exploring set and name a region this robot has not mapped. That it is an
   oversight rather than an invariant is shown by `PlanGlobalConvoy`, which
   guards **both** ends before the identical call. The crash sits on a path
   whose result is discarded (defect 1) — deleting the dead call would
   delete the crash. Hence our admissibility rule above.
5. **A no-show deadlocks permanently.** `AllRobotsInComms`
   (`grid_world.cpp:4916-4926`) requires *every* robot, with no timeout, no
   quorum and no exclusion. One absent peer sends `PlanForRendezvous` down
   the `else` at `:4879` forever, emitting `[cur, cur]` with `wait_ = true`
   while the countdown never resets (it only resets on all-in-comms paths).
   The robot stands still indefinitely. The information to break it is
   computed and ignored: `CheckLostRobot` (`:4347-4401`) sets
   `robots[i].lost_`, and nothing in the rendezvous path ever reads it. The
   5-cycle `wait_at_rendezvous_count_` cap (`:4855-4860`) is **not** the
   no-show handler — it sits inside `AllRobotsInComms` and after
   `AllRobotsSyncedWithRendezvous`, so it only runs once everyone has
   arrived *and* agreed. It is a disperse-after-meeting hold. The
   genuinely-unsynced wait at `:4870-4877` has no counter at all.
6. **Same-shape failure on the initial rendezvous.** Each robot latches its
   own first cell (`grid_world.cpp:285-289`) with no exchange and no
   agreement step; consistency is a deployment assumption ("they start
   together"), unenforced by any code in the tree. If it fails, each robot
   drives to a different cell, `AllRobotsInComms` is never true, and the
   deadlock in (5) follows on the first meeting.
7. **A robot fails the sync test against its own stale advertisement.**
   Self's pair is published at `grid_world.cpp:1967-1974`, and only
   afterwards (`:2039`) does `PlanForRendezvous` run, possibly changing
   `next_*`, before `AllRobotsSyncedWithRendezvous` reads the fresh value
   (`:4935-4936`) and loops over **all** robots with no `i == kRobotID`
   skip (`:4939`). On any cycle where the proposal changes, sync fails for
   that reason alone. This is what freeze-ordering looks like when it is
   wrong, and it is the direct argument for freezing our pair at the last
   *bidirectionally confirmed* snapshot rather than at publish time.
8. **A size guard is inert in both copies** — operator precedence.
   `rendezvous_manager.cpp:46` and `:125` write
   `!x.relay_comms_ordered_cell_ids_.front().size() < 2`, which parses as
   `(!size()) < 2`: a bool compared against 2, always true. The intended
   `!(size() < 2)` never happens, and the following lines index `front()[1]`
   unguarded. The correct form of the same guard exists at
   `grid_world.cpp:4947`. Textbook `[[checks-that-stopped-checking]]`, and
   the reason our equivalents get known-answer calibration.
9. **`AllRobotsSyncedWithRendezvous` mutates negotiation state.** On success
   it calls `ResetPlannedNextRendezvous()` (`:4965-4968`), clearing the
   latch that otherwise freezes a robot's own proposal. Calling the
   predicate for its boolean alone changes behaviour.
10. **Dead state**: `Robot::rendezvous_cell_id_` (`robot.h:81`) is never
    read or written; two dead `robot_cell_id` locals at `grid_world.cpp:1936`
    and `:4881`. Vestiges of an earlier design where the pair had its own
    home before being moved into the relay slot.

### 3.6 Utility-gated reconnection (the mTARE "hybrid")

Today reconnection dispatch is gated by silence timers (the 90 s mid-run
clock, by design — the deliberate info gate). The mTARE gate is economic.
Applies wherever a chase can be armed — `pursuit` and `hybrid` (§3.6.1).
`rendezvous` has no mid-run chase, so there is nothing for it to gate:

1. **Knowledge gate** (port of `HasKnowledgeToShare`): attempt reconnection
   only if some allocator-input or COVERED cell is not in the missing
   peer's `known_by` — meaningful *only* with §3.1 reset semantics.
   Finished peers are ignored.
2. **Value gate** (port of `SolveVRPAssumeComms`, simplified): compute
   `C_no = makespan(no_comms plan)` and `C_re = my reconnect-leg cost +
   makespan(assume_comms plan)`; dispatch iff `C_re < C_no` strictly.
   mTARE samples random peer subsets 3× per cycle; with N≤3 we evaluate
   the missing-peer subsets exhaustively (≤3 subsets) — no sampling.
3. Selected by `reconnect_gate: silence | info` (default `silence` —
   today's exact behaviour). The silence floor is kept even under `info`
   (never dispatch at a peer heard < min-silence ago).

#### 3.6.1 Two mechanisms, four arms (v4 — reverses the v2 arbitration rule)

v2 forbade composing the countdown with the reactive dispatch, on the
grounds that it "would put two triggers on one state machine with
whoever-fires-first semantics". That objection was correct about the
composition v2 had in mind and **does not apply to the one specified here**,
because the departure rule (§3.5) partitions the timeline instead of racing
on it:

    [ contact lost ................ departure deadline ....... t_meet + wait ]
    |<------ CHASE owns this ------>|<---- APPOINTMENT owns this ---------->|

Exactly one mechanism is armed at any instant, and the boundary is a
computed deadline, not an arrival order. That is admissible; the v2 form was
not. Recorded as a reversal so a reader who remembers the v2 rule can see it
was reconsidered rather than forgotten.

The modes are therefore not three strategies but **two independent
mechanisms, each on or off**:

| `reconnect_mode` | chase | appointment | Mid-run trigger | Fallback destination |
| --- | --- | --- | --- | --- |
| `off` | — | — | none | — (today's off arm) |
| `pursuit` | ✓ | — | silence or info gate | none by design; explore-fallback then barrier |
| `rendezvous` | — | ✓ | departure deadline only | the agreed cell |
| `hybrid` | ✓ | ✓ | silence or info gate; chase preempted by the departure deadline | the agreed cell, midpoint as floor |

Terminal (finish-time) dispatch is unchanged in every mode.

Three consequences worth stating explicitly, because each contradicts
something in v2/v3:

- **The chase trigger does not change and is not synchronised.** Each robot
  still dispatches on its own silence/info gate, exactly as today. What
  makes the two robots *arrive together* is the shared `t_meet`, not a
  synchronised start: each departs when its own travel time consumes the
  remaining countdown, so departures stagger and arrivals coincide.
  Synchronising the trigger was considered and rejected — it would require
  the info gate to run on frozen shared state, which is a strictly larger
  change for a property the appointment already provides.
- **`hybrid`'s fallback destination changes** from the last-contact midpoint
  to the agreed cell. §3.7's note that the midpoint "remains" is superseded.
  The midpoint survives as the **floor** in the `penalty` argmin of §3.5:
  it is always available, always agreed, and costs at most half a comms
  range to reach, so when no tour cell is worth its detour the behaviour
  degrades exactly to today's. Hybrid can therefore not come out worse than
  the current implementation on fallback cost.
- **`rendezvous` alone stops meaning "drive to the midpoint on silence".**
  It becomes the pure scheduled strategy: ignore the silence entirely, keep
  exploring, and leave only when the countdown demands it. This is what
  makes the four arms a factorial rather than a menu.

**Why the factorial matters.** mh1 (off vs hybrid, 30/30, one binary
`8121506`) can only support a hybrid-vs-off claim; which half of hybrid does
the work is undetermined by that design. `pursuit` and `rendezvous` alone
are exactly the two missing cells. They must be run **in one campaign
invocation** — `run_campaign.sh` is seed-major, and separate invocations
confound arm with session on a box that drifts ~8%.

**What mh1 says the mechanism should be.** The info gate declined 100 of 110
evaluations, every one on cost (`c_re_mm > c_no_mm`) and none for lack of
knowledge. That is the predicted signature of a worthless destination: the
midpoint is behind both robots, on ground already covered, so reconnecting
can only ever cost more than not reconnecting. Giving the fallback a
tour-side destination lowers `C_re` directly, and the decline rate is the
falsifiable prediction — if it does not move, the tour-relative objective is
wrong and the section should be reverted, not tuned. Completion times for
reference: `off` median 600.0 s / mean 642.4 s / sd 152.5; `mtare_hybrid`
median 676.5 / mean 663.8 / sd 108.3 (n = 30 per arm, all `rc=0`,
all `all_done`). Note hybrid is *tighter* (sd 108 vs 152, better worst case
943 vs 1001) while being slower at the median — the arms differ in
distribution shape, not by a location shift, so a single-number summary of
this pair is misleading whichever statistic is chosen.

### 3.7 `pursuit_predictor` — MDP interception

Straight port of `pursuit_mdp` (181 LOC, Eigen only), feeding the *existing*
pursuit state machine a better waypoint:

- The peer's last **directly received** tour (`my_tour` from its last
  TeamWorld — tours are not gossiped, §3.2) is expanded into a G/I/O node
  chain with mTARE's transition probabilities (params, mTARE defaults);
  forward-propagate to get P(peer at cell c at time t).
- Chase target: over cells on the peer's route, maximise
  P(peer at c at now + my_travel_time(c)) — intercept where they will be,
  not where they were. Re-evaluated each cycle; TOPwTVR is not ported.
- Everything downstream is unchanged: staleness-scaled budget, budget cap,
  and the link vetoes at both dispatch sites. **The fallback ladder gains a
  third terminator** (v4): alongside `pursuit-budget`
  (`explo_planner_node.cpp:7665`) and `trail-exhausted` (`:7704`), a chase
  in `hybrid` also ends at the §3.5 departure deadline, which fires before
  either by construction. Superseding v2/v3: hybrid's fallback destination
  is now the agreed cell, not the last-contact midpoint — see §3.6.1. The
  midpoint remains as the floor of the §3.5 argmin, so a hybrid run with no
  worthwhile tour cell degrades exactly to today's behaviour.
- Degradation: with no tour on record, the trail is today's
  `[peer_goal, peer_pos]` — pursuit never gets *worse* than the current
  implementation.

## 4. Phases

Every phase lands param-gated **default-off**: with all new params at
defaults the binary must be behaviourally identical to its parent commit
(verified per phase — §6). Each phase is a PR-sized unit on
`mtare_evolution` with its own unit tests passing and one 2-robot sim smoke.

**P0 — scaffolding (no behaviour change).**
New empty lib units + test targets in CMake; `TeamWorld.msg` +
`CellState.msg`; `team_robot_names` identity param (§3.0); NDJSON schema
bumped to v4 with new event kinds declared (`cell_census`, `allocation`,
`rendezvous_agreed`, `rendezvous_outcome`, `reconnect_gate`). **P0 also owns
the telemetry-consumer updates**: `sim/gate_g8.py` pins `schema_version == 3`
as part of the pre-registered gate identity, and `sim/event_log.py` reads
schema 3 — both must be updated for v4 *with their known-answer
recalibration re-run* (a deliberate schema regression must still fail).
Until that lands, g8's schema check is *expected* to fail on any v4 run —
that is the gate working, not noise, and loosening the pin instead of
recalibrating is the checks-that-stopped-checking failure mode this plan
explicitly forbids.
Gate: full existing test suite + golden smoke (§6) + recalibrated g8 on a
v4 run.

**P1 — cell world, self only.**
`cell_world` computing status from MapCache; no exchange, no behaviour
coupling. RViz marker layer + `cell_census` NDJSON events.
Gate: unit tests (status transitions, hysteresis, update_id locality,
known_by reset-on-change, guard table); smoke run shows census converging
to COVERED as coverage saturates, agreeing with `coverageUnknownFraction`
to within a cell-quantisation bound.
Realised as `sim/gate_p1.py`, scored on a smoke run's event log. The
agreement bound is *derived* rather than picked:

    roi_unknown_fraction  ≤  R · ( thr · f  +  1 · (1 − f) )

where `f` is the first-hand COVERED fraction, `thr` is
`cell_exploring_min_unknown` — the hysteresis **release** threshold, since
that is what `classify()` actually enforces on an already-COVERED cell, not
the promotion threshold — and `R = grid_area / roi_area ≥ 1`, because the
grid rounds up and can overhang the ROI. `covered_by_others` is excluded
from `f`: it is relayed belief with no local measurement behind it (always
0 in P1, but excluded now so P2 does not fail spuriously).
The gate also checks one **exact** identity — `Σ changed` over all census
rows equals the final `commits_total`, since every change goes through
`commitSelf` and every `applyObservation` emits — which doubles as a
lost-event check. It refuses to pass vacuously: an agreement bound is
trivially satisfied at `f = 0`, so a run that never promotes enough cells
is reported as an explicit **NOT-APPLICABLE failure**, not a pass. Its own
guards are calibrated by `sim/gate_p1_calib.py` (25 injections), and the
`cell_world` assertions by six known-answer source injections.
Default-off equivalence is owned by `sim/equiv_pair.sh`, which verifies the
binary's compile-time-baked `git_rev` out of the run's own event log rather
than trusting that a build happened.

**P2 — TeamWorld exchange + team_model.**
Publish/subscribe TeamWorld through the comms emulator; wire normalisation
+ merge guard table; comms status with handshake + closure; gossip arrays;
N-generalised peer tables (link-gate N=2 refusal replaced). Includes the
cross-repo plumbing (§3.2): `hmr_sim` emulator topic config, planner split
pub/sub params, harness rx/bag/readiness wiring.
Gate: unit tests (merge idempotency; a P3-style serialisation round-trip
**through the wire codec** — normalise, transmit, merge — not through
in-memory state; no-regression property of the guard table under
interleaved lost messages; one-way contact → not IN_COMMS; closure and
gossip freshness with a 3-robot fixture); 2-robot smoke with COMMS=1
showing both cell worlds converging after a dropout heals.

**P3 — global allocator + hierarchy coupling.**
`global_allocator` + focus-neighbourhood filtering in `doPlan`, behind
`global_alloc_enabled`. MinPos, blacklists, proximity stop unchanged.
Gate: unit tests (cross-perspective determinism *through the wire codec*:
serialise robot A's world, merge into B's, and vice versa — identical
allocation on both; mask correctness; tie-break totality; integer cost
quantisation; staleness demotion). Smoke gate is **mechanism-level, not
outcome-level** (the redundancy metric needs ~4.4k cells/arm and cannot
resolve one A/B pair): from `allocation` NDJSON events, while both robots
are in comms, the fraction of cycles with coinciding focus cells must be
below a threshold calibrated on the allocator-off arm. Outcome metrics
wait for P7.

**P4 — utility-gated reconnection.**
Knowledge gate + value gate (`reconnect_gate: info`), dispatch telemetry
extended with `C_no`, `C_re`, and the verdict. Gate: unit tests on the gate
arithmetic including the strictness boundary; **two** smokes, one per failure
direction: (a) peer provably knows everything → zero info-gated dispatches;
(b) a status change committed after separation (peer verifiably missing it)
→ the knowledge gate must fire. (b) is what catches OR-forever-style
over-suppression bugs that (a) silently passes.

**P5 — pre-planned rendezvous from the grid world.**
`rendezvous_scheduler` deriving `(cell, t_meet)` from the allocation output
(§3.5); `reconnect_mode: rendezvous` as a standalone arm; `hybrid`'s
fallback destination repointed from the midpoint to the agreed cell; the
departure deadline added as the third pursuit terminator (§3.7).
**Substantially smaller than the v3 spec** — the proposal/echo/adoption
protocol and its chatter control are deleted, since agreement now follows
from §3.4's cross-process determinism.

Gate: unit tests on (a) the `penalty` argmin including the midpoint floor
winning when no tour cell is worth its detour; (b) `t_meet = maxᵢ τᵢ(c)`
against a hand-computed two-tour case; (c) **cross-perspective identity
through the wire codec** — serialise A's world, merge into B's and vice
versa, and assert *bit-identical* `(cell, t_meet)` on both, which is the
whole agreement argument and must be tested as such, not assumed from
§3.4's allocator test; (d) departure arithmetic on mission-elapsed time,
including the staggered-departure/coincident-arrival property; (e)
admissibility rejecting a cell absent from a peer's `cell_world` or
unreachable on the roadmap; (f) no-show degrading to "resume with the next
schedule armed" and never to a standstill — the direct regression test for
§3.5.1 edge 5.

Smoke: two robots log identical `(cell, t_meet)` in both NDJSON logs before
separation, chase, break off at their own deadlines, and meet.
**Non-vacuity rule**: a smoke in which the appointment never comes due, or
in which the midpoint floor wins every time, is reported as NOT-APPLICABLE,
not as a pass — the §3.5 objective is only exercised when a tour cell
actually wins.

**P6 — MDP pursuit.**
`pursuit_predictor` behind `pursuit_predictor: mdp | trail` (default
`trail`). Gate: unit tests (chain expansion, probability conservation,
argmax intercept vs a hand-computed case, no-tour degradation); smoke under
forced dropout: chase dispatched at the predicted cell, not the stale goal.

The predictor gets **its own arm tokens**, `mtare_pursuit_mdp` and
`mtare_hybrid_mdp` (v4.1), rather than an environment override on the
factorial's four. It is a third mechanism and the 2×2 has no cell for it:
an mdp run under the name `mtare_hybrid` would be indexed as that arm and
pooled with the trail cells by every analysis script. The two tokens are
the exact stacks of the two arms that chase, with the predictor swapped,
so a P6 comparison is one of them against its own trail-named control in
one `run_campaign.sh` invocation. There is deliberately no token for the
arms that never chase — the predictor has nothing to aim there, and the
cell would record a treatment it cannot carry. The node's `arm` stamp
carries the matching `_mdp` suffix so the directory name and the param
dump cannot disagree; `trail` appends nothing, so every earlier run stamps
the byte-identical string it stamped before.

**P7 — N=3 harness + the four-arm factorial.**
Extend `run_explo_sim_rviz.sh` to N robots (namespace loops exist; the N=2
assertions and pairwise link-gate wiring are the work). Then the validation
campaign: the **2×2 factorial of §3.6.1** — `off`, `pursuit`, `rendezvous`,
`hybrid` — with `global_alloc_enabled` and `reconnect_gate` pinned
identically across all four arms, so the only levers are chase on/off and
appointment on/off.

This supersedes v2/v3's single-lever silence-vs-info plan. The reason is
mh1: it establishes hybrid-vs-off on the current binary but **cannot
attribute the effect to either half**, and `pursuit`/`rendezvous` alone are
precisely the two cells that can. Testing the gate instead would answer a
narrower question at the same cost.

Hygiene, all non-negotiable: **one `run_campaign.sh` invocation** (it is
seed-major; separate invocations confound arm with session on a box that
drifts ~8%); one frozen binary across all four arms, verified from each
cell's own `run_manifest.txt` rather than assumed; ~30 cells/arm as the
**floor** not the target (that floor buys 58–84% power for completion time,
and redundancy needs ~4451 cells and is dead as a between-arm metric); the
**exact permutation test**, never a bootstrap — and since C(120,30) cannot
be enumerated, the sampled permutation must first be validated against a
known exact result at small n before it is trusted (`[[lcg-low-bits-bias]]`).

Ordering rationale: P1→P2→P3 is a strict data dependency (cells → exchange →
allocation). P4 needs P3's plan pair. **P5 now depends on P3, not merely on
P2's plumbing** (v4): the rendezvous is derived from the allocator's tours
and inherits its cross-process determinism, so without P3 there is nothing
to derive it from and no agreement argument. Scheduling P5 after P4 also
keeps the reconnection-subsystem changes serial. P6 needs P2's shared tours
(and P3 to make tours non-trivial).

## 5. Explicit non-goals

- **OR-Tools / exact VRP.** The greedy allocator is deliberately weaker;
  mTARE itself caps its solver at 80 ms and warm-starts, i.e. it also runs
  approximate in practice. If tour quality proves limiting, 2-opt depth is
  the dial — a solver dependency is not.
- **TOPwTVR/PBS/CBS** (~2.6k LOC): MDP-argmax interception only.
- **Convoy, RELAY_COMMS status, heterogeneous mobility, rendezvous-tree**
  variant, time-budget return (mission return already exists).
- **3D cells** — 2D cells with 2.5D column stats (ground-robot assumption).
- **Delta-sync bandwidth optimisation** (mTARE `kSyncDelay`): full-state
  messages are affordable at our scale and strictly simpler.
- **Tour gossip** — only positions/last-heard relay at N=3; pursuit of a
  never-directly-heard peer degrades to trail chase.
- **Porting mTARE's rendezvous *implementation*.** §3.5 keeps the idea — a
  pre-agreed cell and time — and replaces the mechanism wholesale; §3.5.1
  records, with file:line, what the reference actually does and why each
  piece is not ported. The mutual exclusion of scheduled and reactive
  triggers is the specific thing v4 does **not** inherit: §3.6.1 composes
  them on a partitioned timeline instead.
- **Splitting the node monolith.** New logic goes in pure libs + thin
  hooks; a structural node split is a separate effort.

## 6. Compatibility, verification, and experiment hygiene

- **Binary generations.** The first behaviour-affecting merge starts a new
  generation, named by its parent git hash in `run_manifest.txt` as usual.
  On the repo's own campaign counter (experiment_log/gate scripts count to
  9) this is **generation 10** — the "four binary generations" list in
  older notes uses a different counter; to avoid a future pooling mistake,
  campaigns reference the manifest hash, never a bare ordinal. Nothing
  measured on this binary pools with feeb74b3 or earlier.
- **Default-off equivalence gate (per phase).** The sim is nondeterministic
  run-to-run under a fixed seed (established: same seed gave 1803 s and
  856 s), so the gate must not compare event *sequences*, row counts, or
  the latch step — those vary under the same binary and the gate would
  either fail spuriously or decay into an inert PASS. It compares
  **run-invariant structure only**: the run_start param dump diff (new
  params present, all at defaults), schema version, gates-armed flags, the
  *set* of NDJSON event kinds emitted (no new kinds may appear at
  defaults), and manifest identity fields. Calibrated in both directions
  before first use: A/A (two same-binary runs must pass) and a
  known-answer injection (a deliberately enabled new event kind must fail).
  **Not run for P5 and P6** (v4.1, operator decision). Both phases are pure
  additions behind a parameter, and neither widens the two things the pair
  actually compares live: the event vocabulary was fixed when schema v4
  landed (`rendezvous_agreed` / `rendezvous_outcome` were declared in P0,
  not by P5, and P6 adds no kind at all), and neither adds a CSV column.
  What remains is the run_start param diff, which `sim/equiv_gate.py`
  computes from the compiled defaults and which `sim/equiv_gate_calib.py`
  now covers for these registry groups by known-answer injection. The cost
  being accepted is stated rather than argued away: nothing *live*
  confirms the default path is untouched for these two phases, so if a
  default-on leak exists it will first appear as a treated-looking control
  in P7's `mtare_off` arm. That is the arm to read first, and a control
  that does not behave like the pre-P5 control invalidates the campaign
  rather than being explained.
- **CSV stays append-only.** New per-step columns appended at the end only:
  `focus_cell_id`, `cells_exploring`, `cells_covered`, `alloc_makespan_sec`
  (sentinel −1 when the allocator is off). Everything richer goes to
  NDJSON. The planning-tick budget watch is the existing `plan_time_ms`
  column.
- **Wire compat.** New message types, additive only; `RobotIntent`
  untouched. Mixed-build fleets fail loudly (unknown type) rather than
  silently.
- **Executor budget.** TeamWorld merge is O(cells) in the tick; allocator
  is bounded (≤400 cells, ≤8 robots, greedy + capped 2-opt); cell-graph
  all-pairs is low milliseconds (§3.1); `plan_time_ms` is the watch.
- **The planning map.** Cell traversability reads the same
  `global_planning_map` topic the fused-map fix mandated — never
  `~/planning_map`.

## 7. Risks

| Risk | Exposure | Mitigation |
| --- | --- | --- |
| Divergent world views → conflicting allocations | Constant under dropout | Tolerated by design (re-solve each cycle); MinPos + blacklists backstop; wire normalisation + guard table bound the divergence to knowledge, not representation |
| Cross-robot status regression via update_id misuse | Reconnect after long separation | Guard table + more-explored conflict rule (§3.2); update_id never compared cross-robot; unit-tested under interleaved loss |
| Knowledge gate goes vacuous (OR-forever) | Every info-gated dispatch decision | known_by reset-on-change semantics (§3.1); P4 smoke (b) exists specifically to catch over-suppression |
| Allocator nondeterminism across robots (float ties) | Silent solve-same breakage | Integer-quantised costs; tie-break totality unit-tested; cross-perspective determinism tested through the wire codec |
| Focus filter starves the local planner (centroid straddle, stale status) | Stall = the known EIG-gap failure mode | 8-neighbourhood admission + tour-advance + unrestricted fallback + staleness demotion (§3.4) |
| Two reconnection triggers fight (departure deadline vs silence/info) | Every `hybrid` outage | Not mutual exclusion but a timeline partition (§3.6.1): the chase owns the span before the deadline, the appointment owns it after. The deadline is a hard terminator ordered *ahead* of `pursuit-budget` and `trail-exhausted`, so the later two can only fire inside the chase's own span |
| Robots compute *different* appointments (agreement-by-construction fails on differing inputs) | Contact loss mid-divergence | Freeze at the last **bidirectionally confirmed** snapshot, not the last received one (§3.5); cross-perspective bit-identity tested through the wire codec (P5 gate c). Residual risk is bounded by the release condition: `teamComplete` is checked every tick, so two nearby-but-unequal destinations still close the range gap |
| N=3 closure declares comms with a data-silent robot | Chain topologies | Closure is status-only; consumers key on gossiped per-robot freshness (§3.3) |
| TeamWorld flooding the executor | Claim-grace episode precedent | Default-off publishing, copy-only callback, merge in tick, `plan_time_ms` watched |
| Scheduled rendezvous under clock skew | Field: offsets of hours | Mission-elapsed time only, never absolute stamps |
| Value gate mis-priced (coarse travel model) | Wrong dispatch decisions | `C_no`/`C_re`/verdict logged per decision; `silence` mode retained as the control arm |
| Schema v4 breaks gate tooling | Every v4 run until fixed | g8/event_log updates + recalibration are in-scope for P0, not deferred |
| New-mode interactions with done-latch/barrier | Deadlock or early finish | RETURN_SYNC/barrier machinery reused unchanged; latch semantics untouched; finished peers leave the vehicle set (§3.4) |

## 8. Decisions taken (so a reviewer can attack them)

1. Cells are 2D over the ROI, 10 m default, column-statistic driven.
2. Full-state TeamWorld, publishing default-off; harness enables per arm.
3. Greedy makespan insertion + optional 2-opt; no solver dependency.
4. Hierarchy coupling is candidate *filtering* over the focus cell + its 8
   neighbours (frontier candidates only), not a utility bias term.
5. Scheduled rendezvous reuses RETURN_NAV/RETURN_SYNC unchanged; no new
   planner states in any phase.
6. `reconnect_gate` and `pursuit_predictor` default to today's behaviour;
   mTARE behaviour is opt-in per arm.
7. N generalised in data structures (uint32 masks, per-peer tables,
   gossip), validated only at N=2 until P7.
8. Merge conflicts resolve by mTARE's guard table plus a more-explored
   ordering (`COVERED > EXPLORING > UNSEEN`) for everything the table
   doesn't decide; update_id is local-only bookkeeping.
9. Fleet identity is the ordered `team_robot_names` param (§3.0), hash-
   checked on the wire; mismatch is fatal at startup, not coerced.
10. The appointment and the reactive chase **compose** rather than exclude
    (reversing v2/v3 and mTARE): they partition one outage's timeline at
    the departure deadline (§3.6.1). The four arms — `off`, `pursuit`,
    `rendezvous`, `hybrid` — are the 2×2 factorial of the two mechanisms,
    which is what makes "which half does the work" answerable.
11. Finished robots leave the allocator's vehicle set, their cells return
    to the pool, and the knowledge gate ignores them.
12. Comms closure classifies status; data consumers key on gossiped
    freshness, never on `IN_COMMS` alone.

## 9. Revision history

- **v4.1** — during P5–P7 implementation. The **default-off equivalence
  pair is dropped for P5 and P6** (§6): neither phase widens the event
  vocabulary or the CSV, so what the live pair would add over the static
  param-diff check is small, and the residual risk is named and assigned
  to P7's `mtare_off` arm rather than left implicit. P6 gains **two arm
  tokens of its own** (§4, P6) instead of an override on the factorial's
  four, and the node's `arm` stamp gains the matching `_mdp` suffix, so
  the predictor cannot be run under a name that would pool it with its own
  control. The harness is N-robot rather than pairwise: the roster is read
  from the scenario the emulator itself reads, and the node's link gate
  means "all links up" rather than "the link" (identical at N=2).
- **v4** — after mh1 (off vs m-tare-hybrid, 30/30, binary `8121506`) and an
  audit of the vendored mTARE reference. **The rendezvous objective
  changed**: from a geometric minimax 1-centre over frontier cells to the
  minimum-makespan-penalty insertion into the allocator's existing tours
  (§3.5) — the old objective optimised a quantity no robot pays and was
  blind to robot positions, and the reference's version excludes them by
  construction. Consequences: the entire proposal/echo/lowest-id-adoption
  protocol and its chatter control are **deleted**, because agreement now
  follows from §3.4's cross-process determinism rather than from
  negotiation; `t_meet` falls out of tour arrival times instead of a tuned
  formula; P5 shrinks and gains a dependency on P3. **The v2 arbitration
  rule is reversed** (§3.6.1): composing the countdown with the reactive
  dispatch is admissible when a computed departure deadline partitions the
  timeline, which is not the whoever-fires-first race v2 rejected. Modes
  re-framed as a 2×2 factorial of chase × appointment, making `pursuit` and
  `rendezvous` alone the two cells that can attribute hybrid's effect —
  which mh1 by construction cannot. Hybrid's fallback destination repointed
  from the last-contact midpoint to the agreed cell, with the midpoint
  retained as the argmin floor so the change cannot make fallback cost
  worse. §3.5.1 added: a ten-item audit of the reference, recording that
  rendezvous is mTARE's *baseline* rather than its method (off by default,
  dead params, zero tests) and cataloguing the specific defects our spec is
  written against — the commented-out departure test, the robot-0 adoption,
  the `exit(1)` on an unreachable cell, the permanent no-show deadlock, the
  self-vs-stale sync failure, and an operator-precedence guard that never
  fires. P7 re-scoped from the single-lever gate comparison to the
  four-arm factorial.
- **v3** — during P1 implementation: §3.1.1 added, recording that the
  mTARE-analogy thresholds were unsatisfiable against this map
  representation (zero cells promoted over a full run), that re-scaling them
  to one run's measured distribution then failed a second run by 0.003, and
  that the cell status thresholds are consequently **world-calibrated knobs
  in the same family as `done_unknown_fraction`** — library defaults keep
  the saturating-map world model, the sim harness overrides them next to its
  own `DONE_UNKNOWN=0.64`. The frontier veto re-typed from an absolute voxel
  count to a scale-free fraction; `cell_census` widened with the order
  statistics that made the diagnosis possible; P1's gate written up with its
  derived agreement bound and its non-vacuity rule.
- **v2** — after round-1 adversarial review (independent reviewer, both
  codebases): added wire normalisation + allocator input set (was a
  solve-same-breaking inconsistency); known_by reset semantics (knowledge
  gate was vacuous as drafted); guard-table merge replacing
  higher-update-id-wins (permitted status regression); gossip arrays +
  freshness rule (N=3 closure was data-blind); mode arbitration table (two
  triggers raced); equivalence gate rebuilt on run-invariant structure
  (sequences are run-variant under this sim); P3 gate made
  mechanism-level (redundancy metric lacks power for one pair); P7 arms
  changed to silence-vs-info with allocator pinned (was unattributable);
  fleet identity param added (numeric ids didn't exist); g8/event_log v4
  updates pulled into P0 scope; rendezvous reframed as redesign with its
  own convergence argument + chatter control; knowledge-port repriced;
  forest cell size / 3D grid / milliseconds / `plan_time_ms` corrections;
  TeamWorld routing cross-repo work budgeted; `finished` consumer defined.
- **v1** — initial draft.
