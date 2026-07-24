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

## 2. `done_coverage_source: "auto"` can mix two metrics in one coverage streak

**Where:** `coverageUnknownFraction` and the `coverage_done_streak_` accumulation
in [`src/explo_planner_node.cpp`](../src/explo_planner_node.cpp) (~L1146); config
key `done_coverage_source` in
[`config/exploration_params.yaml`](../config/exploration_params.yaml).

**Scenario.** Coverage-based termination fires once the ROI unknown fraction
stays below `done_unknown_fraction` for `done_min_consecutive_steps` PLAN cycles
in a row. In `done_coverage_source: "auto"` the measure is scovox 2.5D column
coverage while no planning_map has been received, then switches to the
planning_map 2D unknown fraction once one arrives. `coverage_done_streak_` is
**not** reset on that switch, so a streak can span both metrics.

**Current behaviour.** With `require_planning_map: false` and `auto`, if a
(latched/late/external) planning_map arrives mid-run after the scovox measure
has already banked one or more low-unknown ticks, the remaining ticks are
measured with the planning_map metric. The "N consecutive low-unknown steps"
gate can therefore be satisfied by a streak whose ticks were not all measured
the same way — and the two sources are calibrated differently (a fraction that
means "done" for one need not for the other), so DONE can trigger on a mix.

**Why it is acceptable today / how it is avoided.**

- Field trials do not publish a planning_map, so `auto` resolves to scovox for
  the entire run and the source never switches — the mixed streak cannot occur.
- The config now **pins `done_coverage_source: "scovox"`** (rather than `auto`)
  precisely so there is no source-switching path at all: the streak is always a
  single-source scovox streak. This closes the issue for field use.
- The failure only re-appears if someone sets the source back to `auto` *and* a
  planning_map can appear part-way through a run.

**Cost.** Only in the `auto` + late-planning_map combination: a premature or
mis-calibrated DONE (the map declared saturated on a streak that mixed a scovox
measure with a planning_map measure).

**Possible fix.** Reset `coverage_done_streak_ = 0` whenever the resolved
coverage source changes from the previous tick (track the last-used source and
zero the streak on a transition), so every counted streak is single-source even
in `auto`.

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

- Every robot loads the same `exploration_params.yaml`, so `n_vantages` and
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

**Scenario / current behaviour.** Two edges remain after the
consecutive-streak fix:

- **`confirm_ticks: 1` still needs two scans.** A brand-new track is created
  with `confirm = 1` and the loop `continue`s without an emit check, so the
  emit test only runs when the tree is detected again — one scan later than
  the configured streak of 1 implies.
- **Same-scan double-count.** A track pushed by one detection is immediately
  matchable by a *later* detection in the same scan. If a trunk fragments into
  two clusters within `track_match_radius_m`, the second cluster advances the
  first's streak, and with `confirm_ticks: 2` the track emits after a single
  scan — the debounce is bypassed.

**Why it is acceptable today.**

- The default (and shipped) `confirm_ticks: 2` masks the off-by-one entirely.
- The double-count needs one trunk to yield two clusters inside the match
  radius in one detector pass — rare with the trunk-band clustering — and the
  planner's `target_dedup_radius_m: 1.5` absorbs a duplicate/early TreeTarget
  anyway (id- and proximity-dedup on ingest).

**Cost.** Slightly earlier emission than the documented "N consecutive scans"
contract in the fragmented-trunk case; a latent surprise if `confirm_ticks: 1`
is ever used.

**Possible fix.** Run the emit check on track creation when
`confirm_ticks <= 1`, and mark tracks created in the current scan so same-scan
detections cannot advance their streak (or match against a snapshot of the
track list taken at scan entry).

## 6. Median trunk axis is biased under one-sided observation

**Where:** the axis/radius fit in
[`src/tree_detector.cpp`](../src/tree_detector.cpp) (~L159–181).

**Scenario.** A trunk seen from one side only — the *normal* state before the
vantage circle fills the far side; the occluded half of the trunk shell has no
voxels.

**Current behaviour.** The centre is the per-component median of the observed
trunk voxels' XY. With a one-sided arc the whole distribution lies on the seen
shell, so the median sits on that arc — biased toward the sensor by roughly
the shell offset (~0.1–0.25 m for typical trunk radii) — and the radius
(median distance to that centre) is correspondingly underestimated. The code
comment claims the median is "immune to … a one-sided observation": it
overclaims — a median resists *outliers*, not one-sided *sampling*.

**Why it is acceptable today.**

- The bias is well inside `vantage_visited_tol_m` (0.75 m) and tiny against
  `vantage_standoff_m` (2.0 m), so the vantage ring it seeds is functionally
  the same ring.
- The ring exists precisely to observe the far side; later scans re-estimate
  the centre from fuller coverage.

**Cost.** A slightly off-centre first vantage ring; a comment that promises
more robustness than the estimator delivers.

**Possible fix.** Replace the medians with a proper circle fit (Taubin/Pratt)
on the trunk-band XY — but that is an algorithm change that needs
re-validation on recorded bags, not a patch.

## 7. A non-finite `p_occ` would poison the detection sort

**Where:** `normEntropy` and the deficit sort in
[`src/tree_detector.cpp`](../src/tree_detector.cpp) (~L48–52, ~L247–254).

**Scenario.** The fused map delivers `p_occ = NaN` for a voxel (an upstream
producer bug — no known occurrence).

**Current behaviour.** `normEntropy` clamps `p` off the {0, 1} rails, so the
log-of-zero path is closed — but `std::min`/`std::max` do not sanitise NaN, so
a NaN input passes through, poisons `mean_entropy` → `info_deficit`, and (a)
the NaN deficit compares false against `deficit_thresh`, so the tree silently
reads *well-observed* and is never targeted, and (b) the `std::sort` comparator
on `info_deficit` violates strict weak ordering — formally undefined behaviour.

**Why it is acceptable today.**

- It is contingent on scovox emitting a non-finite probability, which has not
  been observed; the realistic bad inputs (exact 0/1) are already clamped.

**Cost.** None observed; a theoretical UB path guarded only by upstream
correctness.

**Possible fix.** One line in `normEntropy`:
`if (!std::isfinite(p)) return 1.0f;` (treat an undecidable voxel as maximally
uncertain), or drop non-finite voxels at ingest in `buildInput`.
