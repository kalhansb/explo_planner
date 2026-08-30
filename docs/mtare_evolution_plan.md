# Implementation plan: evolving explo_planner into an M-TARE-style distributed explorer

*Status: revised after adversarial review (round 1). Implementation happens on
a new branch `mtare_evolution` off `new_experiments`; this document stays
uncommitted until approved. Reference clone: `mtare_planner_ref/` at the
workspace root (caochao39/mtare_planner, branch `noetic`, COLCON_IGNOREd).*

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
| Rendezvous | reactive: return to last-contact anchor | scheduled: agreed (cell, time) | Add scheduled mode; keep anchor mode as legacy |
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
  the rendezvous minimax.

### 3.5 `rendezvous_scheduler` — planned meetings

**This is a redesign inspired by mTARE, not a port** — mTARE's
implementation carries a piggybacked `[cell, interval]` encoding, a
robot-0-only adoption marked "tmp fix", and a commented-out travel-time
dispatch test. We take the *idea* (agree on the next meeting before
separating; minimax cell; broadcast-and-converge) and specify our own
protocol, which therefore needs its own convergence argument and tests:

- Proposal: rendezvous cell = the allocator-input cell minimising its
  maximum cell-graph distance to all others (minimax 1-centre;
  `nearest`/`farthest` variants behind a param for ablation), time =
  `now + clamp(min_interval + farthest_dist/(2·v̂), min, max)` on the
  mission clock.
- Agreement: proposals ride in `TeamWorld`; everyone adopts the lowest-id
  live proposal; agreed when all live peers echo the same (cell, time).
  **Chatter control**: once agreed, a robot *freezes* its proposal and
  re-proposes only on invalidation (cell demoted to COVERED, time passed,
  or team membership changed) — without the freeze, the lowest-id robot's
  per-cycle re-solve would move the proposal faster than 0.5 Hz echoes can
  converge and "agreed" would flap. Convergence within
  `2/team_world_hz + jitter` after the last invalidation is the unit-tested
  property.
- Execution: countdown against **mission-elapsed time** (robot clocks in
  the field are hours apart; absolute stamps are never compared — the same
  reasoning as claim-TTL-from-local-receipt). When remaining time ≤ my
  travel time to the cell + margin, dispatch through the **existing**
  `startReturnTo → RETURN_NAV → RETURN_SYNC` machinery with the cell
  centroid as destination — arrival tolerance, wait caps, hold escalation,
  release-on-contact invariant, and the barrier reused unchanged.
- On meeting (all peers in comms): agree the next rendezvous, resume after
  a bounded wait. On no-show: the existing wait-cap ladder applies, then
  resume exploring with the next schedule armed — a missed meeting
  degrades to "try again later", never to a deadlock.
- **Arbitration** (see also §3.6): `reconnect_mode: scheduled` **disables
  the mid-run silence/info dispatch entirely** — the countdown is the only
  reconnection trigger in that mode. mTARE makes the same choice (its
  rendezvous and relay-comms modes are an else-if chain and never run
  together). Composing a countdown with the reactive dispatch would put
  two triggers on one state machine with whoever-fires-first semantics.

### 3.6 Utility-gated reconnection (the mTARE "hybrid")

Today reconnection dispatch is gated by silence timers (the 90 s mid-run
clock, by design — the deliberate info gate). The mTARE gate is economic.
Applies in `pursuit`/`hybrid` modes only (never in `scheduled`, per §3.5):

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

Reconnection trigger arbitration, complete table:

| `reconnect_mode` | Mid-run trigger | Terminal trigger |
| --- | --- | --- |
| `rendezvous` / `pursuit` / `hybrid` | silence or info gate (param) | finish-time dispatch, unchanged |
| `scheduled` | countdown only | barrier at the agreed cell |
| `off` | none | none (today's off arm) |

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
  link vetoes, and the fallback ladder — hybrid's fallback destination
  remains the last-contact midpoint (an agreed-rendezvous-cell fallback
  only makes sense if modes composed, which §3.5 forbids; noted as a
  possible future variant, not in scope).
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

**P5 — scheduled rendezvous.**
`rendezvous_scheduler` + `reconnect_mode: scheduled`; proposals in
TeamWorld; execution through existing RETURN_NAV/RETURN_SYNC; mid-run
dispatch disabled in this mode (arbitration table §3.6). Gate: unit tests
(minimax cell choice, lowest-id adoption, freeze/invalidation chatter
control with a simulated 0.5 Hz echo lag, countdown arithmetic on
mission-elapsed time, no-show degradation); smoke: two robots agree
(identical (cell,time) in both NDJSON logs), meet, and re-separate.

**P6 — MDP pursuit.**
`pursuit_predictor` behind `pursuit_predictor: mdp | trail` (default
`trail`). Gate: unit tests (chain expansion, probability conservation,
argmax intercept vs a hand-computed case, no-tour degradation); smoke under
forced dropout: chase dispatched at the predicted cell, not the stale goal.

**P7 — N=3 harness + validation campaign.**
Extend `run_explo_sim_rviz.sh` to N robots (namespace loops exist; the N=2
assertions and pairwise link-gate wiring are the work). Then the first real
campaign on the new binary, **one lever only**: `hybrid + reconnect_gate:
silence` vs `hybrid + reconnect_gate: info`, with `global_alloc_enabled`
pinned identically in both arms (on, since the info gate is the novelty
under test and needs the allocator; the arm difference is the gate alone).
Off-vs-hybrid is *not* re-run here: it was answered on generation 4 (flat,
and that null is real) and re-running it on the new binary answers nothing
about the new mechanisms. ~30 cells/arm minimum per the power floor, exact
permutation test.

Ordering rationale: P1→P2→P3 is a strict data dependency (cells → exchange →
allocation). P4 needs P3's plan pair. P5 needs P2's plumbing; scheduling it
after P4 keeps the reconnection subsystem changes serial. P6 needs P2's
shared tours (and P3 to make tours non-trivial).

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
- **Composing scheduled rendezvous with reactive dispatch** — mutually
  exclusive modes, like mTARE (§3.5/§3.6).
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
| Two reconnection triggers fight (countdown vs silence/info) | Any run in scheduled mode | Modes mutually exclusive (arbitration table §3.6), like mTARE's else-if chain |
| Agreement chatter (proposal moves faster than echoes) | Scheduled mode at 0.5 Hz | Freeze-on-agreed + invalidation-only re-proposal; convergence bound unit-tested |
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
10. `scheduled` mode and the reactive mid-run dispatch are mutually
    exclusive; the arbitration table in §3.6 is exhaustive.
11. Finished robots leave the allocator's vehicle set, their cells return
    to the pool, and the knowledge gate ignores them.
12. Comms closure classifies status; data consumers key on gossiped
    freshness, never on `IN_COMMS` alone.

## 9. Revision history

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
