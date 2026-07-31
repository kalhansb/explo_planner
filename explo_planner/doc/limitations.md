# Known limitations

Deliberate simplifications and known-but-unfixed rough edges in `explo_planner`.
Each entry states the scenario, the current behaviour, why it is acceptable
today, and how it could be addressed if it starts to matter.

## 1. A surplus robot approaches a fully peer-claimed trunk instead of yielding

**Where:** `doExploitPlan` — the `best_idx < 0` branch in
[`src/explo_planner_node.cpp`](../src/explo_planner_node.cpp) (around the
`computeApproachGoal` fallback, ~L2082–2124).

**Scenario.** Coordination + exploitation are on and more robots converge on one
trunk than there are currently-claimable vantages (e.g. 3 vantages, and all
three angles are MinPos-claimed by closer peers, or n_robots ≥ n_vantages). For
the surplus robot every vantage is rejected with reason `minpos` (`rej_minpos`),
so vantage selection yields `best_idx < 0`.

**Current behaviour.** The `best_idx < 0` branch cannot tell *why* no vantage was
selectable — "the map hasn't grown enough to reach a vantage yet" and "every
angle is already owned by a closer peer" both land here. It takes the generic
fallback and drives *toward* the trunk via `computeApproachGoal`, i.e. it heads
for a trunk the team is already fully covering, instead of reverting to
exploration. The wasted travel continues until the team quota closes the trunk
(so the target flips to DONE and the robot reverts to EXPLORE) or the per-target
exploitation timeout fires.

**Why it is acceptable today.**

- It self-heals: the team quota completing, or the per-target wall-clock timeout
  (`exploit_target_timeout_sec`), always ends the wasted approach — the robot
  never gets stuck, only spends some travel.
- With the common field geometry (2 robots, 3 vantages) there is no surplus:
  each robot gets its own angle and the branch is not reached.
- Approaching is not entirely useless — it stages the robot near the trunk, so
  if a peer fails or drops its claim, this robot is positioned to take the freed
  angle.

**Cost.** Surplus robots spend travel time driving at an already-covered trunk
rather than exploring elsewhere; the effect grows as n_robots approaches or
exceeds n_vantages.

**Possible fix.** Distinguish the rejection reasons at the `best_idx < 0` branch:
if there is at least one *valid* vantage (`n_valid > 0`) and the only thing
blocking selection is `rej_minpos` (all valid angles owned by closer peers),
treat the trunk as "handled by the team" for this robot — revert to EXPLORE (or
hold without approaching) instead of driving in. Reserve the `computeApproachGoal`
fallback for the genuine "no vantage reachable/mapped yet" case
(`rej_unreach` / `rej_map` dominate, `n_valid == 0`).

## 2. `done_coverage_source: "auto"` mixed-metric coverage streak (closed by the map-less refactor)

**Where:** `coverageUnknownFraction` and the `coverage_done_streak_` accumulation
in [`src/explo_planner_node.cpp`](../src/explo_planner_node.cpp) (~L1146); config
key `done_coverage_source` in
[`config/shared_params.yaml`](../config/shared_params.yaml).

**Scenario.** Coverage-based termination fires once the ROI unknown fraction
stays below `done_unknown_fraction` for `done_min_consecutive_steps` PLAN cycles
in a row. In `done_coverage_source: "auto"` the measure is the planning_map 2D
unknown fraction whenever a planning_map is present, and scovox 2.5D column
coverage otherwise. `coverage_done_streak_` is **not** reset if that resolution
changes, so a streak that switched sources mid-run would mix two differently-
calibrated metrics.

**Current behaviour.** The resolved source can no longer change mid-run, so a
streak can never mix metrics. With `use_planning_map: false` (the default) the
planning_map is never subscribed: `latest_plan_map_` stays null for the whole run
and `auto` is scovox throughout. With `use_planning_map: true` the planning_map is
a hard startup precondition (WAIT_FOR_MAP blocks until it arrives), so
`latest_plan_map_` is non-null from the first PLAN tick and `auto` is the
planning_map metric throughout. The old best-effort mode — start map-less, bank
scovox ticks, then adopt a late planning_map mid-streak — was the only path that
could switch the source under a live streak, and removing it closed this issue.

**Why it stays closed.**

- The mid-run source switch is gone with the best-effort path: the source is fixed
  for the whole run, so the streak is single-source for **either** value of
  `use_planning_map` and any `done_coverage_source`.
- The planning_map is also off by default (`use_planning_map: false`), so `auto`
  resolves to scovox for the entire run in every shipped config.
- The config additionally **pins `done_coverage_source: "scovox"`**, bypassing the
  `auto` resolution entirely. Belt and suspenders.

**Cost.** None under the current code — the mixed-metric streak is unreachable.
The historical concern was a premature or mis-calibrated DONE (the map declared
saturated on a streak that mixed a scovox measure with a planning_map measure).

**Possible fix (latent robustness).** If a best-effort late-adoption mode is ever
reintroduced, reset `coverage_done_streak_ = 0` whenever the resolved coverage
source changes between ticks, so every counted streak stays single-source even in
`auto`.

## 3. Team dwell-credit assumes an identical vantage ring on every robot

**Where:** the team-quota merge — `mergePeerDwells`
([`src/target_queue.cpp`](../src/target_queue.cpp)) fed from `onPeerExploitIntent`
and the per-tick catch-up in `doExploitPlan`
([`src/explo_planner_node.cpp`](../src/explo_planner_node.cpp), ~L1789 / L1963);
ring geometry in
[`src/vantage_planner.cpp`](../src/vantage_planner.cpp) (`generateVantages`).

**Scenario.** With coordination on, robots exploiting the same trunk broadcast a
`dwelled_mask` — a bitmask of the ring *indices* they captured — and the union
counts toward the team quota. A peer's bit `i` is interpreted with *this* robot's
`ring[i]`. That only lines up if index `i` names the same vantage on every robot.

**Current behaviour.** The ring is `theta = start_angle_rad + i·(2π/n)`, so index
`i`'s bearing is fixed by `n_vantages` and `vantage_start_angle_deg` **alone** —
independent of the trunk center/radius. Bit `i` therefore means the same physical
angle on every robot **provided those two params match across the fleet**. Nothing
in the code checks that they do; a per-robot mismatch would silently credit the
wrong bearings and could declare a trunk complete with an angle no one captured.

**Why it is acceptable today.**

- Every robot loads the same `shared_params.yaml`, so `n_vantages` and
  `vantage_start_angle_deg` are identical fleet-wide and the index→bearing
  mapping agrees. The quota union is correct.
- The mapping is robust to the differences that *do* vary at runtime: per-robot
  center/radius estimates (real with the live tree detector, since each robot
  estimates the trunk independently) only shift a vantage's *position*, not its
  index/bearing. That offset is absorbed by `vantage_visited_tol_m` (0.75 m) when
  a merged angle is marked visited, so the count stays correct. `vantage_standoff_m`
  differences likewise change only radial distance, not bearing.

**Cost.** Only under a deliberate per-robot mismatch of `n_vantages` or
`vantage_start_angle_deg`: silent mis-crediting of the team quota (a trunk closed
on bearings that were never actually observed).

**Possible fix.** Either (a) carry `n_vantages` + `start_angle` on the RobotIntent
and skip the merge (with a warning) when a peer's differ — detects the misconfig
instead of corrupting silently; or (b) broadcast captured *bearings* rather than
bit indices and have each robot map a peer's bearing to its own nearest index,
which tolerates any ring difference. (a) is the small, defensive option; (b) is
the fully general one.

## 4. `max_steps <= 0` stops the planner before it can exploit

**Where:** the step-budget guard at the top of `doPlan`
([`src/explo_planner_node.cpp`](../src/explo_planner_node.cpp), ~L1105).

**Scenario.** Someone sets `max_steps: 0` (or negative) expecting "no
exploration budget — exploit-only run".

**Current behaviour.** The guard `step_ >= max_steps_` fires on the very first
PLAN tick, *before* the pending-target check below it, so the planner
transitions to DONE (and with `done_action: shutdown`, exits) without ever
entering the exploit sub-loop. The guard's comment ("Only reachable with
done_action=idle") is wrong for `max_steps <= 0`.

**Why it is acceptable today.**

- Every shipped config sets `max_steps: 200` (code default 200); nothing uses
  `<= 0`, so the branch is never taken in practice.

**Cost.** An "exploit-only, no exploration budget" configuration is not
expressible via `max_steps: 0` — it silently shuts down instead.

**Possible fix.** Treat `max_steps <= 0` as "unlimited" (skip the guard), or
run the pending-target check before the budget guard. Either way, add a test
that preserves the property the guard was added for: in `done_action: idle`,
an exploit-queue drain past the budget must not sneak an extra exploration hop.

## 5. Tree-track confirmation gate edge cases

**Where:** the `scan()` track loop in
[`src/tree_detector_node.cpp`](../src/tree_detector_node.cpp) (~L197–245).

**Scenario / current behaviour.** Both edges this entry described are now
closed; it is kept as the record of what they were.

- **`confirm_ticks: 1` needed two scans** — a brand-new track was created with
  `confirm = 1` and the loop `continue`d without an emit check, so the emit
  test only ran when the tree was detected again, one scan later than a
  configured streak of 1 implies. Closed by the `EmitGate` refactor: `scan()`
  now creates the track up front and routes *both* the new-track and
  matched-track paths through `stepEmitGate`, so the emit test runs on the
  creating scan too. The default `confirm_ticks: 2` masked this entirely, which
  is why it went unnoticed.
- **Same-scan double-count** — a track pushed by one detection was immediately
  matchable by a *later* detection in the same scan, so a trunk that fragmented
  into two clusters within `track_match_radius_m` advanced one track's streak
  twice and emitted after a single scan. Closed by the `claimed` vector: it is
  index-aligned with `tracks_` and a newly-pushed track is appended already
  marked claimed, so no later detection in the same scan can match it.

**Cost.** None outstanding. The planner's `target_dedup_radius_m: 1.5` was the
backstop for both (id- and proximity-dedup on ingest) and still is.

**Related, now fixed — re-emission after track timeout.** A third edge was
observed on the `map-test-2` bag and has been closed. One trunk was emitted
twice, ~190 s apart, under *different* ids (`813062754` at (14.00, 11.50) and
`823040963` at (14.00, 11.40)): its track aged out at `track_timeout_sec: 30`,
a later detection created a fresh track, and a 0.10 m drift in the centre
estimate straddled an `id_cell_m` boundary so the position hash minted a new
id. Both the emit-once contract and the "same tree ⇒ same id fleet-wide"
contract broke at once, and because the ids differed the planner's id-keyed
dedup could not absorb it either — only its `target_dedup_radius_m` proximity
check stood between this and a double-booked target. `scan()` now keeps an
`emitted_` list of published centres that outlives the tracks; a new track
within `track_match_radius_m` of one adopts that id and starts already
`emitted`. Emit-once is now a property of the tree, not of the track.

## 6. Median trunk axis is biased under one-sided observation (closed by the circle fit)

**Where:** the axis/radius fit in
[`src/tree_detector.cpp`](../src/tree_detector.cpp), now `fitCrossSection` +
the trunk-axis block of `fitAndScore`.

**Scenario.** A trunk seen from one side only — the *normal* state before the
vantage circle fills the far side; the occluded half of the trunk shell has no
voxels.

**Old behaviour.** The centre was the per-component median of the observed
trunk voxels' XY. With a one-sided arc the whole distribution lies on the seen
shell, so the median sat on that arc — biased toward the sensor by roughly
the shell offset — and the radius (median distance to that centre) was
correspondingly underestimated. The code comment claimed the median was
"immune to … a one-sided observation": it overclaimed — a median resists
*outliers*, not one-sided *sampling*.

**The consequence this entry originally missed.** The bias was assessed only
against vantage-ring placement, where it was indeed harmless. But the same
centre is the origin for the **azimuth binning that produces
`angular_coverage`** — the 60%-weighted primary term of `info_deficit`. A
centre that follows the observed voxels sits *inside* the observed shell, so
those voxels fan out around it through every sector and coverage reads ~1.0 for
a trunk seen from one side. The metric went blind exactly when a tree most
needed circling.

On a 300 s replay of the `map-test-2` bag this pinned 13 of 17 real trunks at
`coverage = 1.00` (the rest at 0.88). Since `deficit = 0.60·(1−cov) +
0.25·ent + 0.15·(1−vert)` and `vertical_completeness` is itself structurally
pinned at 1.00, the reachable deficit ceiling was `0.25·ent ≤ 0.25` — below
the 0.35 `deficit_thresh`. **No properly-observed tree could ever be
nominated**, so the exploitation loop could not start. Raising
`n_azimuth_bins` to 16 did not help: coverage still tracked trunk voxel *count*
(0.69 at 112 voxels rising to 1.00 at 658) rather than viewing geometry.

**Fix (applied).** Two changes, both validated on the bag:

1. `fitCrossSection` fits a circle (Taubin) to the trunk residuals instead of
   taking a median. Each z-layer is de-referenced by its own median before the
   layers are pooled, so axis tilt or bend cancels. Taubin specifically, not
   Kåsa: on a 120° test arc Kåsa put the centre 0.28 m off and the radius at
   0.30 m for a true 0.40 m, because Kåsa is badly biased on partial arcs —
   the only regime that matters here. Degenerate sets (flat wall patches, arcs
   too short to constrain a circle) fall back to the old median estimator and
   are flagged `axis_fitted = false`.
2. Voxels within `0.35·radius` of the axis are excluded from azimuth binning:
   near the axis a few centimetres of noise swings the azimuth through a
   half-turn, lighting an arbitrary sector.

Regression coverage: `TreeDetector.OneSidedThickTrunkIsUnderInformed` and
`CircledThickTrunkIsNotUnderInformed`. Note the pre-existing thin-shell tests
(`OneSidedTrunkIsUnderInformed`) passed throughout the broken period — with a
one-voxel-thick arc even a biased centre leaves the azimuths clustered. The bug
only appears once the surface has realistic thickness and noise, which is why
the new tests build the trunk that way.

**Residual limitation.** The fit reduces the 120° test arc from 1.00 to 0.50,
not to the ideal 0.33. Fitting a circle to a partial arc whose radial noise is
a sizeable fraction of its radius (a 0.4 m trunk on a 0.2 m grid — the real
regime) still biases the centre slightly toward the arc. Map-geometry coverage
is therefore a *proxy* and always will be; the node's `use_bearing_coverage`
(default on) measures viewing geometry directly from the robot bearings a trunk
has been observed from and should be preferred whenever TF supplies a pose. On
the post-fix `map-test-2` map the proxy still reads 1.00 on 9 of the 16 detected
trunks — better than the pre-fix 13 of 17, and no longer *unconditionally*
saturated, but not a signal to gate on.

## 7. A non-finite `p_occ` would poison the detection sort (FIXED)

**Where:** `normEntropy` and the deficit sort in
[`src/tree_detector.cpp`](../src/tree_detector.cpp).

**Scenario.** A producer delivers `p_occ = NaN` for a voxel (an upstream bug —
no known occurrence).

**Behaviour before the fix.** `normEntropy` clamped `p` off the {0, 1} rails, so
the log-of-zero path was closed — but `std::min`/`std::max` do not sanitise NaN,
so a NaN input passed through, poisoned `mean_entropy` → `info_deficit`, and
(a) the NaN deficit compares false against `deficit_thresh`, so the tree
silently read *well-observed* and was never targeted, and (b) the `std::sort`
comparator on `info_deficit` violated strict weak ordering — formally undefined
behaviour. Note the occupancy gates (`p_occ < occ_thresh`) do **not** filter it
out: that comparison is false for NaN, so a poisoned voxel is *kept*.

**Fix (applied).** `normEntropy` now returns 1.0 for any non-finite input —
an undecidable voxel reads maximally uncertain, which is the conservative
direction (it raises the deficit rather than hiding the tree). That is the only
NaN inlet into the score: `coverage` and `vertical` are ratios of counts, and
`infoDeficit` already guards a zero weight sum. Covered by
`TreeDetector.NonFinitePOccDoesNotPoisonTheScore`.

**Residual.** Defence in depth only, at the library boundary. The ROS node's
`buildInput` ([`src/tree_detector_node.cpp`](../src/tree_detector_node.cpp))
already computes `p = (N > 0) ? a_occ / N : 0.5`, and `N` is NaN whenever either
Beta count is, so the live path substitutes 0.5 (below `occ_thresh`, hence
dropped) before the detector sees it. The guard matters for any other caller of
the pure library, which constructs `SemVoxel` directly.

## 8. `vertical_completeness` cannot fall below 1.0 for a real trunk

**Where:** the vertical-completeness block of `fitAndScore` in
[`src/tree_detector.cpp`](../src/tree_detector.cpp).

**Scenario.** Any detected tree, in either mode.

**Current behaviour.** The metric bins `members` into `n_height_bins` over
`[base_z, top_z]` and reports the filled fraction. But `base_z` and `top_z` are
*defined* as the min and max z of `members`, so:

- the first and last bins are always occupied, by construction; and
- the span is normalised by its own extent, so the metric is scale-free — a
  1.5 m sapling and a 12 m tree are scored identically.

It can therefore only ever drop below 1.0 on an *interior* gap of at least
`1/n_height_bins` of the observed height containing no voxels at all, which a
continuous trunk never has. It read exactly **1.00 on all 17 trunks** of the
`map-test-2` bag. The name promises an occlusion signal; the computation
delivers an interior-hole detector.

Weighted at `w_vertical: 0.15` this was not merely useless but harmful: because
the weights are auto-normalised, a term stuck at its "fully informed" value
consumed 15% of the deficit range and lowered the ceiling every other term had
to reach.

**Mitigation (applied).** `w_vertical` now defaults to `0.0` (and `w_coverage`
to `0.75`), so the dead term no longer compresses the score. The field is still
computed and reported, because an interior hole *is* worth seeing in the logs —
it just is not vertical completeness.

**Not fixed.** The metric is still misnamed for what it computes. A real
vertical-completeness signal needs an external reference for expected extent
(e.g. bins over `[terrain, terrain + expected_tree_height]`, so a trunk whose
canopy was never observed reads short) rather than the cluster's own bounds.
That conflates "short tree" with "occluded tree" unless the reference is
per-species or learned, which is why it was left alone rather than guessed at.

## 9. Under bearing coverage the emission gate barely discriminates (mitigated by the settle wait)

**Where:** `EmitGate` / `stepEmitGate` in
[`src/tree_detector.cpp`](../src/tree_detector.cpp), driven from the `scan()`
verdict + emit path in
[`src/tree_detector_node.cpp`](../src/tree_detector_node.cpp), with
`use_bearing_coverage: true` (the default).

**Scenario.** Any run. Surfaced while validating the §6 fix.

**Old behaviour.** A track's `bearing_mask` starts empty and only accumulates
from the scan that created the track onward. A tree therefore had **no** bearing
history at its first confirmed detection, so `angular_coverage` read
1/`n_azimuth_bins` (0.12 at the default 8), `info_deficit` landed around
0.72–0.86 against a 0.35 threshold, and the tree was emitted immediately.
Combined with emit-once the practical contract became *"every detected tree is
nominated exactly once, on sight"* — the header describes a gate that separates
under-observed trees from adequately-observed ones, and there was nothing for it
to separate, because a robot cannot have circled a tree the detector had not yet
found. `deficit_thresh` was decorative and `info_deficit` degraded from a gate
to a priority score (the detection sort still uses it, so the neediest tree is
offered first either way).

**Mitigation (applied).** Emission now additionally waits until the tree's
bearing history has gained no new sector for `bearing_settle_ticks` scans
(default 3, i.e. 6 s at the default 2 s cadence) — "we have finished passing
this tree; is it *still* under-covered?". A tree the robot walks around fills
enough sectors to stop reading under-informed and closes itself silently. The
wait is skipped when no pose is available (nothing to settle) and can be
disabled with `bearing_settle_ticks: 0`.

On a clean 300 s `map-test-2` replay this changed the shape of the output:

| measure | before (`settle_ticks: 0`) | after (`settle_ticks: 3`) |
| --- | --- | --- |
| emissions | all in one scan, 0.03 s apart | 24, spread over 526 s of the run |
| trees reading adequately covered | 0 | 4 of 17 at the final scan |
| emitted with >1 bearing sector | 0 | 3 of 24 |

**Not fixed — most trees still fire on a single sector.** 21 of those 24
emissions still carried `cov = 0.12`. The settle counter measures elapsed scans,
not viewing progress, and bearing rate falls off with range: a trunk 20 m away
sweeps well under one 45° sector in 6 s, so it settles and fires long before the
robot gets near it. The wait therefore only bites for trees the robot is
actively walking past — which is exactly the population it was aimed at, but it
means the gate is still permissive for everything else.

**Why that is acceptable today.**

- Nominating a trunk once is a defensible exploitation policy — the planner's
  queue, `target_dedup_radius_m` and the team quota decide what is worth driving
  to. The gate is a filter, not an admission control.
- It is a strict improvement on the pre-§6 behaviour, where the gate admitted
  *no* real tree at all and the loop could not start.
- Emit-once bounds the cost: 24 targets over a 300 s replay, not 24 per scan.

**Cost.** Trees the robot has already observed adequately from a distance still
consume a queue slot and a vantage ring; and every target now costs
`bearing_settle_ticks` scans of latency.

**Possible fix.** Settle on *observation progress* rather than elapsed scans:
also reset the counter when the trunk's voxel count grows by more than a few
percent, so a tree the robot is still approaching (bearing static, map still
filling) does not count as finished. That closes the range-dependent hole above
with one extra knob. The general fix — seeding the mask from observation history
instead of track history — needs scovox to carry a per-voxel first-observation
bearing, which is an upstream map-format change.
