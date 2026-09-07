# explo_planner — real-robot tuning guide

## Executive summary

The planner's motion-dependent timeouts were re-derived from the two robots'
own recordings and shipped as a hardware overlay on the shared parameter
file. With the shipped defaults, a healthy CURT Mini would have failed goals
it was about to reach, because the defaults assume a robot that is always
moving.

The evidence is one recording per robot from 2026-07-31, analysed for speed,
yaw rate, standstill length and travel per time window (§2). Two facts drove
every value. The CURT Mini is stationary half the time and stands still for
up to 22.6 s while healthy, longer than the whole shipped 15 s watchdog
window. Both platforms are skid-steer, so facing a goal costs a 12 to 13 s
turn with zero translation, which the watchdog counts as no progress.

| Parameter | Shipped | Hardware | Basis |
|---|---|---|---|
| `progress_window_sec` | 15 | 30 | clears the longest healthy standstill with margin (§3.2) |
| `progress_min_distance_m` | 0.2 | 0.25 | half the smallest travel seen in any 30 s window (§3.2) |
| `goal_rotate_timeout_sec` | 15 | 25 | one 180° turn plus settling (§3.3) |
| `nav_speed_estimate_mps` | 0.15 | 0.22 bunker, 0.20 curt | measured wall-clock mean, standstills included (§3.4) |
| `base_frame` | `<robot>/base_link` | `base_link`, `base_link_curt` | the default frame exists on neither robot (§3.1) |

The navigation budget's safety factor and caps were deliberately left alone.
The watchdog is what catches a stuck robot, and a tighter budget would only
abort slow hops that were about to succeed. The pose-jump threshold is also
unchanged but unvalidated, since neither recording contained a localisation
correction (§3.5).

Separately from tuning, three settings cannot come from any recording and
must be set before a real run (§4): the ROI box for the site, the peers' pose
topics for the proximity guard, and a finite rendezvous wait if the team may
not reconvene.

---

How the planner's motion-dependent parameters were set for the **Bunker Mini**
(tracked) and the **CURT Mini** (wheeled), what the values rest on, and which
parameters cannot be measured at all and must be set by hand before a real
run. It covers the planner's own parameters only. Bring-up order, the mapper,
the localiser and Nav2 are the [user manual](user_manual.md).

The planner is checked out at `ws/src/explo_planner` inside the
[`hmr_explo`](https://github.com/kalhansb/hmr_explo) workspace repo, which
also holds the recordings, the bag runners and the scovox container used
below. Everything here lands in one file in this repo,
[`config/exploration_real_robot.yaml`](../explo_planner/config/exploration_real_robot.yaml),
an overlay layered on
[`shared_params.yaml`](../explo_planner/config/shared_params.yaml).

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

Both launch files pass parameters to the node in this order, and **later
entries win**:

```
shared_params.yaml  →  params_file:=<overlay>  →  the launch file's own dict
```

The overlay therefore carries only the keys where hardware differs from the
shared file. Anything it does not mention is inherited, so there is one place
to change a shared value and no drift between two copies.

Three blocks, matched by node name:

| Block | Matches | Carries |
|-------|---------|---------|
| `/**` | every planner node | watchdog, rotate deadline |
| `/bunker/explo_planner` | the node namespaced `/bunker` | frames, speed, name, team size, targets topic |
| `/curt/explo_planner` | the node namespaced `/curt` | same for curt |

The node is named `explo_planner`
([explo_planner_node.cpp:654](../explo_planner/src/explo_planner_node.cpp#L654)),
so a per-robot block matches both the two-robot launch (which namespaces each
node by robot) and a hand launch with `-r __ns:=/<robot>`. It does **not**
match the single-robot launch, whose node is the un-namespaced
`/explo_planner`.

```bash
CFG=$(ros2 pkg prefix explo_planner)/share/explo_planner/config

# Two robots, one host
ros2 launch explo_planner multi_robot_exploration.launch.py \
     robots:=bunker,curt params_file:=$CFG/exploration_real_robot.yaml

# One planner per robot PC (the field topology, user manual §6 step 7)
ros2 run explo_planner explo_planner_node --ros-args -r __ns:=/curt \
     --params-file $CFG/shared_params.yaml \
     --params-file $CFG/exploration_real_robot.yaml

# One robot alone
ros2 launch explo_planner exploration_experiment.launch.py \
     robot:=curt base_frame:=base_link_curt \
     params_file:=$CFG/exploration_real_robot.yaml
```

Two consequences of the ordering to keep in mind:

- **The launch dict beats the overlay.** The single-robot launch always
  passes `map_frame`, `base_frame`, `output_csv`, `max_steps`,
  `dscovox_topic` and the trajectory-scoring keys from its arguments; the
  two-robot launch passes `robot_name`, `output_csv`, `max_steps` and the
  four coordination switches. Set those on the command line, not in the
  overlay. That is why the single-robot command above carries `base_frame:=`.
- **A key in the wrong block is silently dropped.** ROS 2 discards
  parameters whose node key does not match, with no warning. Watchdog values
  go under `/**`; anything platform-specific goes under the robot's own block.

---

## 2. How the values were derived

Every timeout in the overlay comes from the robots' own recordings, not from
guesses: the two 2026-07-31 bags in the workspace's `bags/` directory
(bunker `2026_07_31_10_57_06__kalhan_2_`, curt
`2026_07_31_11_24_12__kalhan_2_CURTMINI`), mounted at `/scovox/bags` in the
scovox container. The planner was not running when they were recorded; only
the TF tree was needed.

The analysis is
[`scripts/measure_bag_motion.py`](../scripts/measure_bag_motion.py), written
for this and kept in the repo so it can be repeated after a platform change
or for a new robot. It reads `/tf` and `/tf_static`, composes the base frame
up to the root of its chain, and prints each figure next to the parameter it
sizes. Static transforms are applied before the dynamic pass, because the
curt bag records its `map_global → map_curt` transform partway through and
applying it in stream order showed up as a fake 3 m jump.

```bash
# inside the scovox container, where rosbag2_py is installed
python3 scripts/measure_bag_motion.py /scovox/bags/2026_07_31_11_24_12__kalhan_2_CURTMINI base_link_curt
python3 scripts/measure_bag_motion.py /scovox/bags/2026_07_31_10_57_06__kalhan_2_ base_link --only-3d
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
| min travel in any 30 s window | 1.33 m | 0.51 m |

**What the figures say.** CURT is the binding constraint on every timeout: it
is stationary half the time and stands still for up to 22.6 s at a stretch on
a healthy run. Both robots share the `/**` block, so every shared value is
sized for CURT and merely generous for bunker. Both platforms are skid-steer
and cannot strafe: they stop and rotate in place to face a goal, and an
in-place rotation translates zero metres, so every distance-over-time check
has to tolerate at least one full 180° turn of no translation on a perfectly
healthy robot.

**What was wrong in the bunker recording.** The script's `EDGES` section
exists because of it. Read plainly, the bunker bag gives a 60 Hz sample rate
and a p99 single-tick jump of 49 m: two publishers were writing the
`odom → base_link` edge, a 10 Hz 3D one and a 50 Hz planar one, disagreeing
by about 6 m. Every figure derived from such a bag is nonsense. The bunker
column above was recovered with `--only-3d`, which drops the planar
publisher's samples by their signature (`z == 0`, yaw-only rotation). That
reads an old recording; it does not fix the platform, and the overlay header
carries the fault as a blocker for any real run.

**What the bags could not exercise.** The `map → odom` correction was
constant in both recordings (odometry only, no loop closure), so
`max_pose_jump_m` has never met a real relocalisation and is carried as
unvalidated (§3.5). And since the planner was not driving the robots, no bag
produced a single failure tag; the first run where those mean anything is the
validation run (§5).

---

## 3. Parameters

Each entry gives what the parameter does in the node, the shipped value
against the hardware value, and how the hardware value follows from §2.
"Inherited" means deliberately left at `shared_params.yaml`.

### 3.1 `base_frame`, `map_frame`

Left empty, `base_frame` defaults to `<robot_name>/base_link`
([explo_planner_node.cpp:668](../explo_planner/src/explo_planner_node.cpp#L668)).
On these robots the frames are `base_link` (bunker) and `base_link_curt`
(curt), so the default names a frame that exists on neither. The lookup
fails, the planner never acquires a pose, and it sits without ever reaching
PLAN. Nothing else in the overlay matters until this is right.

| | bunker | curt |
|---|---|---|
| `base_frame` | `base_link` | `base_link_curt` |
| `map_frame` | `map` | `map`, or `map_curt` if the two maps are not fused |

The startup line to check is `EIG exploration planner ready: … frame=map
base=base_link_curt`. If it prints `base=curt/base_link`, the overlay did not
reach the node (wrong block, or the single-robot launch without
`base_frame:=`).

### 3.2 `progress_window_sec`, `progress_min_distance_m`

The no-progress watchdog
([explo_planner_node.cpp:2305](../explo_planner/src/explo_planner_node.cpp#L2305)).
On entering NAVIGATE the planner marks the current cumulative distance. Every
`progress_window_sec` it checks how much path has been walked since the mark;
if that is under `progress_min_distance_m` it fails the goal with
`[no-progress]` and blacklists it (§3.6); otherwise it re-marks and starts the
next window. The check is on the **total path walked**, not on distance
closed to the goal, so a robot rotating in place or waiting for Nav2 to
re-plan scores zero even when nothing is wrong.

It is deliberately not applied during the final in-place rotate at the goal,
but the turn-to-face at the *start* of a hop happens inside the first window.

| | shipped | hardware |
|---|---|---|
| `progress_window_sec` | 15.0 | **30.0** |
| `progress_min_distance_m` | 0.2 | **0.25** |

**Why the shipped values are unsafe here.** CURT's longest healthy standstill
(22.6 s) is longer than the whole 15 s window. A healthy robot trips the
watchdog, the goal it is standing next to is blacklisted for a minute, and
the planner walks off to a worse one.

**How the hardware values were chosen.**

- *Window*: longer than the longest healthy standstill plus one 180° turn.
  22.6 s + 13 s ≈ 36 s in the worst case, but those two never coincided in
  the recordings; 30 s clears the standstill with margin and leaves 17 s of
  driving time after a full turn.
- *Distance*: about half the smallest travel measured in any window of that
  length (0.51 m over 30 s), so a genuinely wedged robot still fails within
  one window while a slow or turning one never does.

**Re-tuning.** From the script's `STILL` line and `WINDOW` table: pick the
window first, then set the distance at roughly half the travel figure for
that window. Raising the window costs nothing but reaction time on a truly
stuck robot: it takes one full window to notice, then the Nav2 recovery on
the next goal.

### 3.3 `goal_rotate_timeout_sec`

The rotate deadline
([explo_planner_node.cpp:676](../explo_planner/src/explo_planner_node.cpp#L676)).
Once the robot is inside `goal_xy_tolerance` and only the yaw remains, a
separate deadline starts. Overrunning it fails the goal with
`[budget-rotate]` — and blacklists a goal the robot has **already reached**.

| | shipped | hardware |
|---|---|---|
| `goal_rotate_timeout_sec` | 15.0 | **25.0** |

A 180° turn takes 12–13 s at the p90 yaw rate. The shipped 15 s leaves two
seconds for Nav2 to settle. 25 s is one full turn plus a comfortable margin.
Re-tune from the script's `180 deg turn at p90` figure: one turn plus about
ten seconds.

`goal_xy_tolerance` and `goal_yaw_tolerance` (0.4 / 0.4) must stay looser
than Nav2's goal checker, or arrival is never registered and this deadline
fires on every goal. That interaction is in the user manual §5.

### 3.4 `nav_speed_estimate_mps` and the distance budget

`nav_speed_estimate_mps`, `nav_safety_factor`, `nav_min_timeout_sec`,
`nav_max_timeout_sec`
([explo_planner_node.cpp:704](../explo_planner/src/explo_planner_node.cpp#L704),
budget set at
[:2075](../explo_planner/src/explo_planner_node.cpp#L2075)).

```
budget = clamp( distance / nav_speed_estimate_mps × nav_safety_factor,
                nav_min_timeout_sec, nav_max_timeout_sec )
```

`distance` is the straight line to the goal, or the planned path length when
a planning map is in use. A hop that outlives its budget fails with
`[budget]`.

| | shipped | bunker | curt |
|---|---|---|---|
| `nav_speed_estimate_mps` | 0.15 | **0.22** | **0.20** |
| `nav_safety_factor` | 3.0 | inherited | inherited |
| `nav_min_timeout_sec` | 30.0 | inherited | inherited |
| `nav_max_timeout_sec` | 180.0 | inherited | inherited |

With curt's values a 3 m hop gets 45 s, an 8 m hop 120 s, and anything past
12 m the 180 s cap.

**The speed is the wall-clock mean, not a moving speed.** The script's
`LINEAR mean` is path over total time, standstills included. That is the
number the budget needs: CURT is stationary 49 % of the time, so a budget
built from its speed while moving would abort healthy hops. The safety factor
of 3 is then real headroom for detours, not a correction for waiting.

**Do not tighten the budget to catch a stuck robot.** The watchdog (§3.2)
does that job within one window. Shortening the budget only aborts slow hops
that were about to succeed. If `[budget]` fires on goals the robot was still
visibly approaching, raise `nav_safety_factor` or the cap; leave the speed at
the measurement.

Raising the cap has one coupling: `exploit_target_timeout_sec` (300 s) keeps
charging across failed hops and must stay above `nav_max_timeout_sec`.

### 3.5 `max_pose_jump_m`

Pose-jump rejection
([explo_planner_node.cpp:719](../explo_planner/src/explo_planner_node.cpp#L719),
applied in `trackDistance` at
[:3949](../explo_planner/src/explo_planner_node.cpp#L3949)).
Any single-tick pose step larger than this is treated as a localisation
correction rather than travel and is **not** added to the cumulative
distance. At the 10 Hz tick, 1.0 m means 10 m/s, so it cannot reject real
motion: the fastest measured sample (2.26 m/s) moves 0.23 m per tick.

| | shipped | hardware |
|---|---|---|
| `max_pose_jump_m` | 1.0 | inherited, **unvalidated** |

Unvalidated because the bags never exercised it (§2). Two ways it can
misbehave in the field:

- **Set too low** — real motion is discarded, the cumulative distance
  freezes, and the watchdog (§3.2) fails a robot that is driving perfectly.
  Keep it well above the max speed × 0.1 s.
- **A broken TF edge** — two publishers on `odom → base` (the bunker case in
  §2) make the pose flicker by metres every tick. Every step is swallowed as
  a "teleport", the distance freezes, and the watchdog fires as above. The
  symptom in the log is a stream of `Pose jump of X m in one tick exceeds
  max_pose_jump_m` warnings while the robot is plainly moving. That is a
  platform fault, not a parameter to tune.

After the validation run, grep the log for `Pose jump of`. Rare hits at
moments the localiser corrected are fine. Continuous hits mean the TF edge.

### 3.6 `failed_goal_radius_m`, `failed_goal_ttl_sec`

The failed-goal blacklist, inherited at 2.0 m / 60 s. Every `[no-progress]`,
`[budget]` or `[budget-rotate]` failure parks a 2 m disc around the goal for
60 s. This is why a false watchdog trip costs more than one hop: the region
the robot is standing in becomes off-limits and it drives somewhere worse.
Tune the three timeouts above until failures are rare and genuine before
touching either of these. Three failures on the same spot is a panic-stop
trigger (user manual §9), not a tuning problem.

---

## 4. Required to update before real runs

No recording can supply these. They are wrong by default and **fail
quietly**, so they are set before the first run at a site, not tuned
afterwards.

### ROI box — `roi_min_x` … `roi_max_y`

Candidates outside the box are discarded
([explo_planner_node.cpp:785](../explo_planner/src/explo_planner_node.cpp#L785)),
and the coverage-done test measures the unknown fraction over the box only.
The shipped values (−51.3…100.9 × −38.7…74.5) are the extent of a **previous
site**. A box that misses the survey area starves the planner of candidates
and it declares done with the site unexplored; a box far larger than the
reachable area puts a permanent floor under the unknown fraction and
coverage-done never fires. Both robots must carry the identical box.
`roi_min_z` / `roi_max_z` (−5.5 / 4.0) are robot-relative and should span
ground to canopy.

### Peer poses — `proximity_peer_pose_topics`

The proximity guard runs on the 1 Hz intent heartbeat unless this lists the
peers' localiser poses. Entries are `"<robot_name>:<topic>"`, the topic a
`PoseWithCovarianceStamped` in the map frame
([explo_planner_node.cpp:1195](../explo_planner/src/explo_planner_node.cpp#L1195)).
A bare topic without the `name:` prefix is skipped with a warning.

Why the heartbeat is not enough on these platforms: the hold distance is
5 m, and between two heartbeats the pair can close by

| closing speed | per heartbeat |
|---|---|
| both at p90 (0.51 + 0.60 m/s) | 1.1 m |
| both at max (1.99 + 2.26 m/s) | 4.3 m |

At the worst case that is nearly the whole hold distance unseen. A 30 Hz pose
brings it to under 0.15 m.

It is left unset in the overlay rather than guessed, because a wrong topic
name produces exactly the same silent never-holds behaviour as no topic at
all. The NDT pipeline publishes its pose as `/pcl_pose`; confirm the
namespaced name on the platform with `ros2 topic info -v`, then set in each
robot's block:

```yaml
/bunker/explo_planner:
  ros__parameters:
    proximity_peer_pose_topics: ["curt:/curt/pcl_pose"]
/curt/explo_planner:
  ros__parameters:
    proximity_peer_pose_topics: ["bunker:/bunker/pcl_pose"]
```

The startup line confirms it: `Proximity stop: tracking peer 'curt' via
/curt/pcl_pose`. The same startup block prints `cancel via
'/curt/navigate_to_pose'` — that must be the platform's Nav2 action name
(`proximity_nav_cancel_action`), or a hold cancels nothing and the robot keeps
driving while the planner believes it has stopped.

### Team size — `rendezvous_expected_peers`, `rendezvous_max_wait_sec`

Rendezvous is inert at the shipped `expected_peers: 0`
([explo_planner_node.cpp:891](../explo_planner/src/explo_planner_node.cpp#L891)).
The overlay's per-robot blocks set it to 1, and the two-robot launch sets it
from the team size anyway. The single-robot launch matches neither, so a
robot running alone keeps it at 0 — which is right: a robot that has never
heard a peer has no anchor and simply finishes.

`rendezvous_max_wait_sec` inherits 0.0, which means **wait forever**. A robot
that exhausts its goals after losing comms drives back to where it last heard
its teammate and holds there until the team is complete. Correct for a
controlled trial; a field hazard if the team may not reconvene. Set a finite
give-up if that is a possibility.

### Calibrated from the validation run

`done_unknown_fraction` (0.05) and `candidate_enable_polar` (true) are site
behaviour, not motion, and can only be set once the planner has driven the
site. The user manual §5 says what to watch for both.

---

## 5. Reading the validation run

The validation run is the first run on the real robots with this overlay. The
bags never had the planner driving (§2), so it is the first time a failure
tag means anything. Each failure line names its cause:

```
Step 12: navigation failed [no-progress] after 31.2s at goal (14.20, -3.10). Blacklisted; 2 active failed-goal entries.
```

| Tag | The robot was … | Then adjust |
|-----|-----------------|-------------|
| `[no-progress]` | driving or turning normally | raise `progress_window_sec`, or lower `progress_min_distance_m` (§3.2) |
| `[no-progress]` | genuinely wedged | nothing — it worked; a shorter window only if it took too long to notice |
| `[budget]` | still approaching the goal | raise `nav_safety_factor` or `nav_max_timeout_sec`; keep the speed (§3.4) |
| `[budget-rotate]` | at the goal, still turning | raise `goal_rotate_timeout_sec` (§3.3); check the tolerances against Nav2 |
| `[budget-rotate]` | at the goal, not moving | tolerances tighter than Nav2's goal checker (user manual §5) |

Warnings that mean a setting did not take:

| Line | Meaning |
|------|---------|
| `frame=map base=curt/base_link` | overlay not applied, or `base_frame:=` missing on the single-robot launch (§3.1) |
| `Pose jump of … exceeds max_pose_jump_m` continuously | broken TF edge (§3.5), not a parameter |
| `Proximity stop: no peer pose topics configured` | peer poses unset (§4) |
| `proximity_peer_pose_topics entry '…' is not '<robot_name>:<topic>'` | missing the `name:` prefix |
| `Rendezvous inactive (… expected_peers=0)` | team size not set for this node |

In the metrics CSV, `distance_traveled` should climb whenever the robot
moves. A flat trace while it is driving means the pose is not being tracked
(§3.1) or every step is being rejected (§3.5); either way the watchdog will
fire next.

---

## 6. Trying a change against a bag first

The bag runners at the root of the `hmr_explo` workspace
(`run_explo_dual.sh`, `run_explo_curtmini.sh`) layer
[`config/exploration_fused_bag.yaml`](../explo_planner/config/exploration_fused_bag.yaml),
which **disables** the watchdog (`progress_min_distance_m: 0.0`). A recording
cannot react to the planner's goals, so the robot in the bag never drives
toward them and the watchdog would fail every goal. Bag replay therefore
cannot validate the watchdog by observation; only by measurement, which is
what the script in §2 is for.

What a replay does validate, before any battery is spent:

- the `frame=… base=…` startup line and pose acquisition,
- `distance_traveled` climbing as the recorded robot moves, and the absence
  of `Pose jump` warnings,
- candidate counts inside the ROI box, and where the unknown fraction
  plateaus for `done_unknown_fraction`.

To check the hardware overlay itself against a bag, add it as a second
`--params-file` before the bag overlay, so the bag file's `use_sim_time` and
watchdog-off still win.

---

## 7. Checklist

Before the first run on a new platform:

- [ ] run `scripts/measure_bag_motion.py` on a recording of the robot driving
- [ ] `odom → base` edge has one publisher (script's `EDGES` section)
- [ ] `base_frame` in the robot's block; `base=` in the startup line matches
- [ ] `progress_window_sec` > longest standstill + one 180° turn
- [ ] `progress_min_distance_m` ≈ half the min travel at that window
- [ ] `goal_rotate_timeout_sec` > one 180° turn + 10 s
- [ ] `nav_speed_estimate_mps` = wall-clock mean

Before the first run at a new site (§4):

- [ ] ROI box set to the survey extent, identical on both robots
- [ ] `proximity_peer_pose_topics` set with the `name:` prefix; startup line shows `tracking peer`
- [ ] `proximity_nav_cancel_action` matches the platform's Nav2 action name
- [ ] `rendezvous_expected_peers` = team size − 1 on every node that is part of the team
- [ ] `rendezvous_max_wait_sec` decided
- [ ] after the validation run: `done_unknown_fraction` set above the measured plateau
