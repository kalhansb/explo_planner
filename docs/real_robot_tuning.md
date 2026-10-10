# explo_planner — real-robot tuning guide

## Executive summary

[`config/exploration_real_robot.yaml`](../explo_planner/config/exploration_real_robot.yaml)
is the hardware overlay for the **Bunker Mini** (tracked) and the **CURT
Mini** (wheeled). It sits on top of
[`shared_params.yaml`](../explo_planner/config/shared_params.yaml) (§1). The
shared timeouts assume a robot that keeps moving. CURT stands still for up to
22.6 s on a healthy run, and both robots turn in place for 12 to 13 s to face
a goal, so the overlay sizes the timeouts from their own recordings (§2):

| Parameter | `shared_params.yaml` | Overlay | § |
|---|---|---|---|
| `base_frame` | `""` → `<robot>/base_link` | `base_link` (bunker), `base_link_curt` (curt) | 3.1 |
| `progress_window_sec` | 15 | 120 | 3.2 |
| `progress_min_distance_m` | 0.2 | 0.25 | 3.2 |
| `goal_rotate_timeout_sec` | 15 | 25 | 3.3 |
| `nav_speed_estimate_mps` | 0.15 | 0.22 (bunker), 0.20 (curt) | 3.4 |
| `nav_max_timeout_sec` | 180 | 600 | 3.4 |
| `exploit_target_timeout_sec` | 300 | 720 | 3.4 |

It also turns on the planning map and sets each robot's name, team size, CSV
path and topics (§1).

Before a real run, fix bunker's doubled `odom → base_link` TF edge (§2), and
set what no recording can supply (§4): the ROI box, the peers' pose topics
and the rendezvous wait.

Bring-up, the mapper, the localiser and Nav2 are in the
[user manual](user_manual.md).

**Contents**

- [Executive summary](#executive-summary)
1. [How the overlay is layered](#1-how-the-overlay-is-layered)
2. [How the values were derived](#2-how-the-values-were-derived)
3. [Parameters](#3-parameters)
4. [Required to update before real runs](#4-required-to-update-before-real-runs)
5. [Reading the validation run](#5-reading-the-validation-run)
6. [Trying a change against a bag first](#6-trying-a-change-against-a-bag-first)
7. [Checklist](#7-checklist)

---

## 1. How the overlay is layered

Both launch files pass parameters in this order. **Later entries win**:

```
shared_params.yaml  →  params_file:=<overlay>  →  the launch file's own dict
```

The overlay holds only the keys where hardware differs. The rest is
inherited.

| Block | Matches | Carries |
|-------|---------|---------|
| `/**` | every planner node | watchdog, rotate deadline, nav budget cap, exploit target timeout, planning map |
| `/bunker/explo_planner` | the node in namespace `/bunker` | name, frames, speed, CSV path, team size, targets and planning-map topics |
| `/curt/explo_planner` | the node in namespace `/curt` | same for curt |

The planning-map keys are explained in [field_setup.md](field_setup.md),
"The planning map" and "Traversability map".

The node is named `explo_planner`
([explo_planner_node.cpp:926](../explo_planner/src/explo_planner_node.cpp#L926)).
A per-robot block therefore matches the two-robot launch and a hand launch
with `-r __ns:=/<robot>` (the field topology, user manual §6 step 7). It
does **not** match the single-robot launch, whose node `/explo_planner` has
no namespace. The run commands for each case are in
[field_setup.md](field_setup.md), "Which config for which run".

Two rules follow:

- **The launch dict beats the overlay.** Each launch file sets the keys
  behind its own arguments ([ros_api.md](ros_api.md) §6). The two-robot
  launch also sets `robot_name`, `output_csv` and
  `rendezvous_expected_peers`. Set these through the launch. That is why the
  single-robot launch needs `base_frame:=` as an argument.
- **A key in the wrong block is silently dropped.** ROS 2 ignores
  parameters under a node key that does not match. Shared values go under
  `/**`, platform values under the robot's block.

---

## 2. How the values were derived

The source is one recording per robot from 2026-07-31: bunker
`2026_07_31_10_57_06__kalhan_2_` and curt
`2026_07_31_11_24_12__kalhan_2_CURTMINI`. They are in the `bags/` directory
of the [`hmr_explo`](https://github.com/kalhansb/hmr_explo) workspace repo,
mounted at `/scovox/bags` in the scovox container. Only their TF tree is
used. The planner was not running.

[`scripts/measure_bag_motion.py`](../scripts/measure_bag_motion.py) reads
the TF tree and prints each figure next to the parameter it sizes. Rerun it
after a platform change or for a new robot:

```bash
# inside the scovox container, where rosbag2_py is installed
python3 scripts/measure_bag_motion.py /scovox/bags/2026_07_31_11_24_12__kalhan_2_CURTMINI base_link_curt --windows 30,60,90,120
python3 scripts/measure_bag_motion.py /scovox/bags/2026_07_31_10_57_06__kalhan_2_ base_link --only-3d --windows 30,60,90,120
```

| | Bunker Mini (tracked) | CURT Mini (wheeled) |
|---|---|---|
| mean speed (wall-clock) | 0.223 m/s | 0.203 m/s |
| p90 / p99 speed | 0.510 / 0.816 m/s | 0.603 / 1.459 m/s |
| max speed | 1.99 m/s | 2.26 m/s |
| yaw rate p90 | 0.241 rad/s | 0.261 rad/s |
| 180° turn at p90 | 13.0 s | 12.0 s |
| stationary fraction | 18 % | 49 % |
| longest standstill | 1.9 s | 22.6 s |
| min net displacement in any 30 s window | 0.00 m | 0.00 m |
| 30 s windows under 0.25 m net | 10 % | 9 % |
| 60 / 90 / 120 s windows under 0.25 m net | 0 / 0 / 0 % | 6 / 2 / 0 % |
| min net displacement in any 120 s window | 3.23 m | 2.33 m |

**CURT sets every timeout**, because it stands still far more and both
robots share the `/**` block. Both robots are skid-steer, so they turn in
place to face a goal and move 0 m doing it. Every distance-over-time check
must allow at least one 180° turn with no translation.

**The bunker recording has a TF fault.** Two publishers write its
`odom → base_link` edge: a 10 Hz 3D one and a 50 Hz planar one, about 6 m
apart. Read as-is, the bag gives a p99 single-tick jump of 49 m and wrong
figures throughout. `--only-3d` drops the planar samples (`z == 0`,
yaw-only rotation) so the old recording can be read. The platform itself
must still be fixed: the overlay header lists the fault as a blocker for
real runs, and the script's `EDGES` section reports it.

**Not tested by the bags.** `map → odom` was constant in both (odometry
only), so `max_pose_jump_m` is unvalidated (§3.5). The planner was not
driving, so there are no failure tags yet. The validation run (§5) is the
first test of both.

---

## 3. Parameters

Values are in the summary table. "Inherited" means left at
`shared_params.yaml` on purpose.

### 3.1 `base_frame`, `map_frame`

Left empty, `base_frame` defaults to `<robot_name>/base_link`
([explo_planner_node.cpp:940](../explo_planner/src/explo_planner_node.cpp#L940)),
a frame neither robot has. The TF lookup fails, the planner never gets a
pose, and it never reaches PLAN. Nothing else matters until this is right.
Bunker uses `base_link`, curt `base_link_curt`. `map_frame` is `map` on
both, or `map_curt` on curt if the two maps are not fused.

Check the startup line `EIG exploration planner ready: … frame=map
base=base_link_curt`. `base=curt/base_link` means the overlay did not reach
the node: wrong block, or the single-robot launch without `base_frame:=`.

### 3.2 `progress_window_sec`, `progress_min_distance_m`

The no-progress watchdog
([explo_planner_node.cpp:3097](../explo_planner/src/explo_planner_node.cpp#L3097)).
At the end of each `progress_window_sec` in NAVIGATE, the planner measures
the straight-line distance from where the robot was when the window opened.
Under `progress_min_distance_m`, it fails the goal with `[no-progress]` and
blacklists it (§3.6).

This is **net displacement**, so a healthy robot turning in place or
waiting for Nav2 to re-plan scores near zero. The final turn at the goal is
exempt. The turn to face the goal at the start of a hop is not.

**Window: 120 s.** CURT's longest healthy standstill (22.6 s) is longer than
the shipped 15 s window, so the watchdog would blacklist goals a healthy
robot was about to reach. The watchdog is a backstop. Nav2 ends most
failures first:

- A goal Nav2 rejects fails on `nav2_abort_limit` within seconds.
- A wedged robot trips Nav2's progress checker (0.5 m in 10 s). Its
  recoveries end in an abort after about 2.5 to 3 minutes, and the planner
  fails the goal on that abort (`nav2_abort_final_after_recovery`).

Only the watchdog ends a stall Nav2 does not report: Nav2 not running or not
watched, or Nav2 reporting SUCCEEDED short of the planner's tolerance. So
the window is sized never to fail a healthy robot. 120 s is the shortest
window in which no stretch of either recording falls under 0.25 m net,
operator stops included (§2). A stall is caught 120 to 240 s after it
starts.

**Distance: 0.25 m.** Far below the 2.33 m minimum net displacement at
120 s. It is kept because it is also the "same spot" radius for
`nav2_abort_limit`.

**Re-tuning.** From the script's `STILL` line and `WINDOW` table, pick the
window, then set the distance to about half its `net min`. Use a recording
driven to goals throughout, ideally by the planner. Operator stops in a
teleop recording pull `net min` to 0, as in both 2026-07-31 bags.

### 3.3 `goal_rotate_timeout_sec`

The rotate deadline
([explo_planner_node.cpp:3060](../explo_planner/src/explo_planner_node.cpp#L3060)).
Once the robot is inside `goal_xy_tolerance` and only the yaw remains, a
separate deadline starts. Overrunning it fails the goal with
`[budget-rotate]` and blacklists a goal the robot has **already reached**.

A 180° turn takes 12 to 13 s at the p90 yaw rate, which leaves the shipped
15 s only 2 s to settle. 25 s is one turn plus a margin. To re-tune, take
the script's `180 deg turn at p90` and add about 10 s.

`goal_xy_tolerance` / `goal_yaw_tolerance` (0.4 / 0.4) must stay looser than
Nav2's goal checker, or this deadline fires on every goal (user manual §5).

### 3.4 `nav_speed_estimate_mps` and the distance budget

The per-hop budget
([explo_planner_node.cpp:2749](../explo_planner/src/explo_planner_node.cpp#L2749)):

```
budget = clamp( distance / nav_speed_estimate_mps × nav_safety_factor,
                nav_min_timeout_sec, nav_max_timeout_sec )
```

`distance` is the straight line, or the planned path's length if a planning
map is in use and the path is longer. A hop that outlives its budget fails
with `[budget]`. `nav_safety_factor` (3.0) and `nav_min_timeout_sec` (30 s)
are inherited. With curt's values a 3 m hop gets 45 s, an 8 m hop 120 s and
a 15 m hop 225 s. Only goals past 40 m (44 m on bunker) reach the 600 s cap.

**Speed: the wall-clock mean.** The script's `LINEAR mean` includes
standstills. CURT is stationary 49 % of the time, so a budget built from its
moving speed would abort healthy hops.

**Cap: 600 s.** Nav2 and the watchdog end most bad goals first (§3.2). The
budget is left with a robot that keeps moving but never arrives, such as one
circling an obstacle: Nav2 replans with no overall time limit, and its
progress checker stays satisfied. A cap of a few minutes would fail long
healthy hops. 600 s still bounds the circling case.

**Do not tighten the budget to catch a stuck robot.** That only aborts slow
hops that were about to succeed. If `[budget]` fires on goals the robot was
still approaching, raise `nav_safety_factor` or the cap. Leave the speed at
the measurement.

**Keep `exploit_target_timeout_sec` above the cap.** It keeps charging
across failed hops, so below the cap a tree closes PARTIAL after its first
failed vantage approach. 720 s is one failed hop (600 s) plus the shipped
120 s margin.

### 3.5 `max_pose_jump_m`

Pose-jump rejection, in `trackDistance`
([explo_planner_node.cpp:5033](../explo_planner/src/explo_planner_node.cpp#L5033)).
A single-tick pose step above this counts as a localisation correction, not
travel. It is not added to `distance_traveled`, and the watchdog's start
point moves with it. At the 10 Hz tick, 1.0 m means 10 m/s. The fastest
measured sample (2.26 m/s) moves 0.23 m per tick, so real motion is safe.

Inherited at 1.0 and **unvalidated** (§2). Two ways it can go wrong:

- **Set too low.** Real motion is discarded and the watchdog fails a robot
  that is driving normally. Keep it well above max speed × 0.1 s.
- **Broken TF edge.** Two publishers on `odom → base` (the bunker fault)
  make the pose flick by metres every tick, with the same result. The log
  shows a stream of `Pose jump of X m in one tick (…) exceeds
  max_pose_jump_m` while the robot is plainly moving. Fix the platform, not
  the parameter.

After the validation run, grep the log for `Pose jump of`. Rare hits at
localiser corrections are fine. Continuous hits mean the TF edge.

### 3.6 `failed_goal_radius_m`, `failed_goal_ttl_sec`

The failed-goal blacklist, inherited at 2.0 m / 60 s: every failed goal
blocks a 2 m disc for 60 s. A false watchdog trip therefore costs more than
one hop, because the area the robot is in becomes off-limits. Tune the
timeouts above before touching these. Three failures on the same spot is a
panic-stop trigger (user manual §9), not a tuning problem.

---

## 4. Required to update before real runs

No recording can supply these. They are wrong by default and **fail
quietly**. User manual §5 has the operator's list. This section gives the
reasons.

### ROI box — `roi_min_x` … `roi_max_y`

Candidates outside the box are discarded
([candidate_generator.cpp](../explo_planner/src/candidate_generator.cpp)),
and coverage-done measures the unknown fraction inside it only.
`shared_params.yaml` ships the tree block of the hmr_localisation gt_map
site: 100 × 25 m, yawed 25.75°. Seven keys set it: `roi_yaw_deg`,
`roi_origin_x`, `roi_origin_y`, and `roi_min/max_x/y` in the yawed frame.
`roi_yaw_deg: 0` with origin (0, 0) gives a plain map-frame box.

A box that misses the survey area starves the planner, and it declares done
with the site unexplored. A box much larger than the reachable area leaves a
floor under the unknown fraction, and coverage-done never fires. To fit the
box to the ground-truth map, see [field_setup.md](field_setup.md), "Area of
interest from the ground-truth map".

### Peer poses — `proximity_peer_pose_topics`

Unset, the proximity guard runs on the 1 Hz intent heartbeat. The hold
distance is 5 m, and between heartbeats the pair can close by:

| closing speed | per heartbeat |
|---|---|
| both at p90 (0.51 + 0.60 m/s) | 1.1 m |
| both at max (1.99 + 2.26 m/s) | 4.3 m |

The localiser's 30 Hz pose cuts the worst case to under 0.15 m.

Entries are `"<robot_name>:<topic>"`, a map-frame
`PoseWithCovarianceStamped`
([explo_planner_node.cpp:1680](../explo_planner/src/explo_planner_node.cpp#L1680)).
An entry without the `name:` prefix is skipped with a warning. The overlay
leaves the key unset, because a wrong topic name fails as silently as none.
The NDT pipeline publishes `/pcl_pose`. Confirm the namespaced name with
`ros2 topic info -v`, then set:

```yaml
/bunker/explo_planner:
  ros__parameters:
    proximity_peer_pose_topics: ["curt:/curt/pcl_pose"]
/curt/explo_planner:
  ros__parameters:
    proximity_peer_pose_topics: ["bunker:/bunker/pcl_pose"]
```

On bunker, startup then logs `Proximity stop: tracking peer 'curt' via
/curt/pcl_pose` and `cancel via '/bunker/navigate_to_pose'`. The cancel
action (`proximity_nav_cancel_action`) must be the platform's Nav2 action,
or a hold cancels nothing and the robot keeps driving. The default matches
only when Nav2 runs in the robot's namespace ([nav2_config.md](nav2_config.md)).

### Team size — `rendezvous_expected_peers`, `rendezvous_max_wait_sec`

Rendezvous is inert at the shipped `rendezvous_expected_peers: 0`. The
overlay's per-robot blocks set 1, and the two-robot launch sets it to the
team size minus one. The single-robot launch matches neither block, so a
robot alone keeps 0. That is right: with no teammate, it finishes where
exploration ends.

`rendezvous_max_wait_sec` is 600 s: from `shared_params.yaml` on a hand
launch, from the `rendezvous_max_wait_sec:=` argument on the two-robot
launch. When it runs out, a robot waiting at its start point parks in `DONE`
(user manual §3.3). 0 waits forever. Use that only in a controlled trial
with someone watching.

### Calibrated from the validation run

`done_unknown_fraction` (0.05) and `candidate_enable_polar` (false) depend
on the site, not on motion, so they can only be set once the planner has
driven it. User manual §5 says what to watch for both.

---

## 5. Reading the validation run

The validation run is the first real-robot run with this overlay, and the
first in which failure tags mean anything. Each failure line names its
cause:

```
Step 12: navigation failed [no-progress] after 121.4s at goal (14.20, -3.10), robot at (11.80, -2.40) 2.50 m short. Blacklisted; 2 active failed-goal entries.
```

| Tag | The robot was … | Then adjust |
|-----|-----------------|-------------|
| `[no-progress]` | driving or turning normally | raise `progress_window_sec` or lower `progress_min_distance_m` (§3.2) |
| `[no-progress]` | genuinely wedged | nothing. Shorten the window only if it took too long to notice |
| `[budget]` | still approaching the goal | raise `nav_safety_factor` or `nav_max_timeout_sec` (and `exploit_target_timeout_sec` with it). Keep the speed (§3.4) |
| `[budget]` | circling or re-routing, never closing in | nothing. This is the case the cap is for |
| `[budget-rotate]` | at the goal, still turning | raise `goal_rotate_timeout_sec` (§3.3) |
| `[budget-rotate]` | at the goal, not moving | tolerances tighter than Nav2's goal checker (user manual §5) |
| `[nav2-aborted]` with `0 recovery(ies)` | parked, Nav2 refused the goal twice | nothing if rare. The `nav2 abort reason` line (Jazzy) names it. Many `GOAL_OUTSIDE_MAP` (204): Nav2's global costmap is too small (field setup, "Goals outside nav2's global costmap") |
| `[nav2-aborted]` with recoveries | wedged. Nav2 ran its recoveries and gave up | nothing. Many on the same kind of ground: the planning map lets goals onto it (field setup, "Traversability map") |

Every goal, reached ones included, also ends with one `Goal end` line. The
CSV has the same fields (`goal_outcome`, `nav_elapsed_sec`,
`goal_path_length_m`, `nav2_aborts`, `nav2_recoveries`, `nav2_error_code`).
Tabulate them to see outcomes per goal kind, budget used and Nav2
recoveries.

```
Goal end [nav2-aborted] step 12 explore (14.20, -3.10): 6.1s of 177s budget, path 11.8 m, 11.40 m short; nav2 ABORTED, 2 abort(s), 0 recovery(ies), last error 204 GOAL_OUTSIDE_MAP
```

Lines that mean a setting did not take:

| Line | Meaning |
|------|---------|
| `frame=map base=curt/base_link` | overlay not applied, or `base_frame:=` missing on the single-robot launch (§3.1) |
| `Pose jump of … exceeds max_pose_jump_m`, continuously | broken TF edge (§3.5) |
| `proximity_peer_pose_topics entry '…' is not '<robot_name>:<topic>'` | missing the `name:` prefix (§4) |

For the rendezvous and proximity warnings, see user manual §6, "Startup
lines that confirm a correct launch".

In the metrics CSV, `distance_traveled` should climb whenever the robot
moves. A flat trace while it drives means the pose is not tracked (§3.1) or
every step is rejected (§3.5), and the watchdog fires next.

---

## 6. Trying a change against a bag first

The bag runners (`run_explo_dual.sh`, `run_explo_curtmini.sh`, in `ws/src/`
of the `hmr_explo` repo) load
[`config/exploration_fused_bag.yaml`](../explo_planner/config/exploration_fused_bag.yaml),
which **turns the watchdog off** (`progress_min_distance_m: 0.0`). A
recording does not drive toward the planner's goals, so the watchdog would
fail them all. Replay cannot test the watchdog. The §2 script measures for
it instead.

A replay does check, before any battery is spent:

- the `frame=… base=…` startup line and pose acquisition,
- `distance_traveled` climbing as the recorded robot moves, with no
  `Pose jump` warnings,
- candidate counts inside the ROI box, and where the unknown fraction
  levels off (for `done_unknown_fraction`).

To check the hardware overlay against a bag, add it as an extra
`--params-file` before the bag file, so the bag file's `use_sim_time` and
watchdog-off still win.

---

## 7. Checklist

Before the first run on a new platform:

- [ ] run `scripts/measure_bag_motion.py` on a recording of the robot driving
- [ ] `odom → base` edge has one publisher (script's `EDGES` section)
- [ ] `base_frame` in the robot's block; `base=` in the startup line matches
- [ ] `progress_window_sec` > longest standstill + one 180° turn
- [ ] `progress_min_distance_m` ≈ half the `net min` at that window, from a recording driven to goals throughout
- [ ] `goal_rotate_timeout_sec` > one 180° turn + 10 s
- [ ] `nav_speed_estimate_mps` = wall-clock mean

Before the first run at a new site (§4):

- [ ] ROI box set to the survey extent, identical on both robots
- [ ] `proximity_peer_pose_topics` set with the `name:` prefix; startup line shows `tracking peer`
- [ ] `proximity_nav_cancel_action` matches the platform's Nav2 action name
- [ ] `rendezvous_expected_peers` = team size − 1 on every node that is part of the team
- [ ] `rendezvous_max_wait_sec` decided
- [ ] after the validation run: `done_unknown_fraction` set above the measured plateau
