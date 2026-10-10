# Known limitations

Deliberate simplifications and known rough edges in `explo_planner`. Each open
entry gives the scenario, the current behaviour, why it is acceptable, and a
fix. Closed entries are kept short, as a record of what went wrong.

## 1. A surplus robot approaches a fully peer-claimed trunk instead of yielding

**Where:** `doExploitPlan`, the `best_idx < 0` branch, in
[`src/explo_planner_node.cpp`](../src/explo_planner_node.cpp).

**Scenario.** Coordination and exploitation are on, and more robots converge on
one trunk than it has claimable vantages (for example n_robots ≥ n_vantages).
Peers already claim every angle, so the surplus robot rejects every vantage as
`rej_minpos` and selection returns `best_idx < 0`.

**Current behaviour.** That branch cannot tell "no vantage is reachable yet"
from "peers own every angle". In both cases it drives toward the trunk via
`computeApproachGoal` instead of returning to exploration. The approach lasts
until the team quota closes the trunk or `exploit_target_timeout_sec` fires. A
robot that has already dwelled on this trunk does not reach this branch:
`holdDwelledVantage` parks it on its own vantage.

**Why it is acceptable.**

- It always ends: the team quota or the per-target timeout stops it. The robot
  loses travel time but never gets stuck.
- The field setup (2 robots, 3 vantages) has no surplus robot. The cost grows
  as n_robots approaches or exceeds n_vantages.
- The robot ends up near the trunk, ready to take an angle a peer drops.

**Fix.** In the `best_idx < 0` branch, if `n_valid > 0` and `rej_minpos` is
the only blocker, treat the trunk as handled by the team: revert to EXPLORE, or
hold without approaching. Keep the `computeApproachGoal` fallback for "nothing
reachable or mapped yet" (`n_valid == 0`, mostly `rej_unreach` / `rej_map`).

## 2. `done_coverage_source: "auto"` mixed-metric coverage streak (closed by the map-less refactor)

**Closed.** In `auto`, coverage-done uses the planning_map unknown fraction
when a planning_map is present and scovox column coverage otherwise.
`coverage_done_streak_` is not reset when that choice changes, so a planning_map
arriving mid-run could mix two differently calibrated metrics in one streak and
trigger an early DONE.

Removing the best-effort mode (start map-less, adopt a late planning_map)
closed it. With `use_planning_map: false` the planning_map is never subscribed.
With `true` it is a WAIT_FOR_MAP precondition. Either way the source is fixed
for the whole run. `shared_params.yaml` also pins
`done_coverage_source: "scovox"`. If late adoption is ever reintroduced, reset
`coverage_done_streak_` whenever the resolved source changes.

## 3. Team dwell-credit assumes an identical vantage ring on every robot

**Where:** `mergePeerDwells` in [`src/target_queue.cpp`](../src/target_queue.cpp),
fed by `onPeerExploitIntent` and the per-tick merge in `doExploitPlan`
([`src/explo_planner_node.cpp`](../src/explo_planner_node.cpp)). The ring comes
from `generateVantages` in [`src/vantage_planner.cpp`](../src/vantage_planner.cpp).

**Scenario.** With coordination on, robots exploiting the same trunk broadcast
`dwelled_mask`, a bitmask of the ring indices they captured. The union counts
toward the team quota. This robot reads a peer's bit `i` as its own `ring[i]`.

**Current behaviour.** Vantage `i` sits at bearing
`vantage_start_angle_deg + i·360°/n_vantages`, whatever the trunk centre and
radius. So bit `i` names the same bearing on every robot only if `n_vantages`
and `vantage_start_angle_deg` match across the fleet. Nothing checks this. A
mismatch silently credits the wrong bearings and can close a trunk on an angle
nobody captured.

**Why it is acceptable.**

- Every robot loads the same `shared_params.yaml`, so both values match.
- Values that do differ per robot (trunk centre and radius from each robot's
  own estimate, `vantage_standoff_m`) move a vantage's position, not its
  bearing. `vantage_visited_tol_m` (0.75 m) absorbs the position offset.

**Fix.** Either (a) carry `n_vantages` and the start angle on `RobotIntent` and
skip the merge, with a warning, when a peer's differ. This is small and
detects the misconfiguration. Or (b) broadcast captured bearings instead of
indices and map each to the nearest local index. This tolerates any ring
difference.

## 4. `max_steps <= 0` stops the planner before it can exploit

**Where:** the step-budget guard at the top of `doPlan` in
[`src/explo_planner_node.cpp`](../src/explo_planner_node.cpp).

**Scenario.** `max_steps: 0` (or negative), meant as "no exploration budget,
exploit only".

**Current behaviour.** The guard `step_ >= max_steps_` fires on the first PLAN
tick, before the pending-target check, and ends exploration through
`finishOrRendezvous`. With `done_action: shutdown` the node exits without
exploiting. With `done_action: idle` (the field default) DONE-idle still picks
up pending targets, by the same past-budget path a run takes once its budget is
spent. The guard's comment ("Only reachable with done_action=idle") is wrong
for `max_steps <= 0`.

**Why it is acceptable.** Nothing uses it: `shared_params.yaml` and the launch
defaults set 200 (100 in `multi_robot_exploration.launch.py`).

**Fix.** Treat `max_steps <= 0` as unlimited, or run the pending-target check
before the budget guard. Add a test that keeps what the guard exists for: with
`done_action: idle`, draining the exploit queue past the budget must not start
an extra exploration hop.

## 5. Tree-track confirmation gate edge cases

**Closed.** Three edges in the `scan()` track loop of
[`src/tree_detector_node.cpp`](../src/tree_detector_node.cpp), all fixed:

| Edge | What happened | Closed by |
| --- | --- | --- |
| `confirm_ticks: 1` needed two scans | A new track skipped the emit check on the scan that created it. The default `confirm_ticks: 2` hid this. | `scan()` creates the track first and runs new and matched tracks through `stepEmitGate`. |
| Same-scan double count | A later detection in the same scan could match a track just created, so a trunk split into two clusters within `track_match_radius_m` advanced its streak twice and emitted after one scan. | The `claimed` vector: a new track is appended already claimed. |
| Re-emission after track timeout | On the `map-test-2` bag one trunk was emitted twice, ~190 s apart, as `813062754` and `823040963`. Its track aged out (`track_timeout_sec: 30`), a new track formed, and a 0.10 m centre drift crossed an `id_cell_m` boundary, giving a new id that the planner's id dedup could not catch. | `scan()` keeps an `emitted_` list that outlives tracks. A new track within `track_match_radius_m` of an emitted centre adopts that id and starts already emitted. |

The planner's proximity dedup (`target_dedup_radius_m: 1.5`) remains the
backstop.

## 6. Median trunk axis is biased under one-sided observation (closed by the circle fit)

**Closed, with a residual limitation.** The axis fit is `fitCrossSection` and
the trunk-axis block of `fitAndScore` in
[`src/tree_detector.cpp`](../src/tree_detector.cpp).

**What it was.** The trunk centre was the median XY of the observed trunk
voxels. For a trunk seen from one side, that median sits on the seen arc. The
same centre is the origin of the azimuth binning behind `angular_coverage`, so
the observed voxels fanned out through every sector and coverage read ~1.0. On
a `map-test-2` replay 13 of 17 trunks read `coverage = 1.00`, `info_deficit`
could not reach `deficit_thresh` (0.35), and no tree was ever nominated, so
exploitation never started.

**What closed it.**

1. `fitCrossSection` fits a Taubin circle to the trunk residuals, after
   centring each z-layer on its own median so tilt or bend cancels. Degenerate
   sets (flat patches, very short arcs) fall back to the median and are flagged
   `axis_fitted = false`.
2. Voxels within `0.35·radius` of the axis are left out of azimuth binning:
   near the axis, centimetres of noise swing the azimuth by half a turn.

Tests: `TreeDetector.OneSidedThickTrunkIsUnderInformed` and
`CircledThickTrunkIsNotUnderInformed`. They build a thick, noisy shell because
the older thin-shell `OneSidedTrunkIsUnderInformed` passed even with the bug.

**Residual.** Map-geometry coverage is still only a proxy. A 120° test arc reads
0.50, not the ideal 0.33, and the post-fix `map-test-2` map still reads 1.00 on
9 of 16 trunks. Do not gate on it. Keep `use_bearing_coverage` on (the
default), which measures coverage from the robot bearings a trunk was seen
from, whenever TF supplies a pose.

## 7. A non-finite `p_occ` would poison the detection sort (FIXED)

**Fixed.** A NaN `p_occ` (an upstream bug, never observed) passed through
`normEntropy`, because `std::min`/`std::max` do not remove NaN. The occupancy
gate did not drop it either (`p_occ < occ_thresh` is false for NaN). The NaN
`info_deficit` made the tree read well-observed, so it was never targeted, and
broke the strict weak ordering of the deficit sort (undefined behaviour).

`normEntropy` in [`src/tree_detector.cpp`](../src/tree_detector.cpp) now
returns 1.0 (maximally uncertain) for any non-finite input. Test:
`TreeDetector.NonFinitePOccDoesNotPoisonTheScore`. The ROS node's `buildInput`
([`src/tree_detector_node.cpp`](../src/tree_detector_node.cpp)) already turns a
NaN Beta count into `p = 0.5`, so the guard matters only for code that calls
the library directly.

## 8. `vertical_completeness` cannot fall below 1.0 for a real trunk

**Where:** the vertical-completeness block of `fitAndScore` in
[`src/tree_detector.cpp`](../src/tree_detector.cpp).

**Scenario.** Every detected tree.

**Current behaviour.** The metric bins the tree's voxels into `n_height_bins`
between `base_z` and `top_z` and reports the filled fraction. Both bounds are
the tree's own min and max z, so:

- the first and last bins are always filled; and
- the score ignores tree height: a 1.5 m sapling scores like a 12 m tree.

It drops below 1.0 only for an interior gap of at least `1/n_height_bins` of
the height, which a continuous trunk never has. It read 1.00 on all 17 trunks
of the `map-test-2` bag. It is an interior-hole detector, not the occlusion
signal its name promises.

**Why it is acceptable.** `w_vertical` defaults to 0.0 (and `w_coverage` to
0.75), so the term does not affect the score. Do not raise it: the weights are
auto-normalised, so a term stuck at "fully informed" only lowers the deficit
ceiling the other terms must reach. The field is still computed and logged,
because an interior hole is worth seeing.

**Fix (not done).** Bin against an external height reference, for example
`[terrain, terrain + expected_tree_height]`, so a trunk whose canopy was never
seen reads short. That confuses short trees with occluded ones unless the
reference is per-species or learned, so it was left alone.

## 9. Under bearing coverage the emission gate barely discriminates (mitigated by the settle wait)

**Where:** `EmitGate` / `stepEmitGate` in
[`src/tree_detector.cpp`](../src/tree_detector.cpp), called from `scan()` in
[`src/tree_detector_node.cpp`](../src/tree_detector_node.cpp), with
`use_bearing_coverage: true` (the default).

**Scenario.** Any run.

**Current behaviour.** A track's `bearing_mask` starts empty when the track is
created. At its first confirmed detection a tree has one bearing sector, so
`angular_coverage` reads 1/`n_azimuth_bins` (0.12 at the default 8) and
`info_deficit` (0.72–0.86) is far above `deficit_thresh` (0.35). On its own,
the gate would nominate every detected tree once, on sight.

The mitigation: emission also waits until the tree's bearing history has gained
no new sector for `bearing_settle_ticks` scans (default 3, i.e. 6 s at the
default 2 s `scan_period_sec`). A tree the robot walks around fills enough
sectors to stop reading under-informed and is never emitted. The wait is
skipped when no pose is available. `bearing_settle_ticks: 0` disables it.

On a 300 s `map-test-2` replay:

| measure | `bearing_settle_ticks: 0` | `bearing_settle_ticks: 3` |
| --- | --- | --- |
| emissions | all in one scan, 0.03 s apart | 24, spread over 526 s of the run |
| trees reading adequately covered at the final scan | 0 | 4 of 17 |
| emitted with >1 bearing sector | 0 | 3 of 24 |

Most trees still fire on a single sector: 21 of the 24 emissions had
`cov = 0.12`. The settle counter counts scans, not viewing progress, and distant
trees change bearing slowly. A trunk 20 m away sweeps well under one 45° sector
in 6 s, so it settles and fires long before the robot is near. The wait only
bites for trees the robot is passing.

**Why it is acceptable.**

- Nominating each trunk once is a defensible policy. The gate is a filter, not
  admission control: the planner's queue, `target_dedup_radius_m` and the team
  quota decide what is worth driving to.
- Emit-once bounds the cost: 24 targets over the replay, not 24 per scan.
- Detections are still sorted by `info_deficit`, so the neediest tree is
  offered first.

The cost: trees already well observed from a distance still take a queue slot
and a vantage ring, and every target waits `bearing_settle_ticks` scans.

**Fix.** Settle on observation progress, not elapsed scans: also reset the
counter when the trunk's voxel count grows by more than a few percent, so a
tree the robot is still approaching does not count as settled. The general
fix, seeding the mask from observation history rather than track history,
needs scovox to carry a per-voxel first-observation bearing (an upstream
map-format change).
