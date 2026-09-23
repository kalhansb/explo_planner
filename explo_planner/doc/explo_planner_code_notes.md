# explo_planner — code comment notes

Long comments from files in the `explo_planner` package, moved out of the code on 2026-09-23 so the sources carry short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word. Files with many moved comments have their own notes doc next to this one.

One section per source file, in file order. Each entry names the function (or section) the comment sat in, the line of code it was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [CMakeLists.txt](#cmakeliststxt) — 8
- [cmake/StampGitRev.cmake](#cmakestampgitrevcmake) — 3
- [config/targets_flatforest.yaml](#configtargets_flatforestyaml) — 1
- [config/targets_map_test2.yaml](#configtargets_map_test2yaml) — 1
- [include/explo_planner/candidate_generator.hpp](#includeexplo_plannercandidate_generatorhpp) — 4
- [include/explo_planner/cost_grid.hpp](#includeexplo_plannercost_gridhpp) — 3
- [include/explo_planner/exchange_drain.hpp](#includeexplo_plannerexchange_drainhpp) — 1
- [include/explo_planner/failed_goal_blacklist.hpp](#includeexplo_plannerfailed_goal_blacklisthpp) — 5
- [include/explo_planner/fleet_identity.hpp](#includeexplo_plannerfleet_identityhpp) — 5
- [include/explo_planner/fov_evaluator.hpp](#includeexplo_plannerfov_evaluatorhpp) — 3
- [include/explo_planner/home_trail.hpp](#includeexplo_plannerhome_trailhpp) — 3
- [include/explo_planner/map_cache.hpp](#includeexplo_plannermap_cachehpp) — 4
- [include/explo_planner/meeting_attendance.hpp](#includeexplo_plannermeeting_attendancehpp) — 2
- [include/explo_planner/proximity_guard.hpp](#includeexplo_plannerproximity_guardhpp) — 3
- [include/explo_planner/pursuit_predictor.hpp](#includeexplo_plannerpursuit_predictorhpp) — 5
- [include/explo_planner/reconnect_gate.hpp](#includeexplo_plannerreconnect_gatehpp) — 3
- [include/explo_planner/separation.hpp](#includeexplo_plannerseparationhpp) — 6
- [include/explo_planner/target_queue.hpp](#includeexplo_plannertarget_queuehpp) — 5
- [include/explo_planner/tree_detector.hpp](#includeexplo_plannertree_detectorhpp) — 9
- [include/explo_planner/vantage_planner.hpp](#includeexplo_plannervantage_plannerhpp) — 2
- [launch/exploitation_experiment.launch.py](#launchexploitation_experimentlaunchpy) — 1
- [launch/multi_robot_exploration.launch.py](#launchmulti_robot_explorationlaunchpy) — 8
- [src/candidate_generator.cpp](#srccandidate_generatorcpp) — 3
- [src/cell_world.cpp](#srccell_worldcpp) — 11
- [src/coordination.cpp](#srccoordinationcpp) — 7
- [src/cost_grid.cpp](#srccost_gridcpp) — 3
- [src/exchange_drain.cpp](#srcexchange_draincpp) — 6
- [src/failed_goal_blacklist.cpp](#srcfailed_goal_blacklistcpp) — 5
- [src/fov_evaluator.cpp](#srcfov_evaluatorcpp) — 2
- [src/map_cache.cpp](#srcmap_cachecpp) — 4
- [src/metrics_logger.cpp](#srcmetrics_loggercpp) — 7
- [src/plan_map_query.cpp](#srcplan_map_querycpp) — 2
- [src/planner_util.cpp](#srcplanner_utilcpp) — 4
- [src/proximity_guard.cpp](#srcproximity_guardcpp) — 1
- [src/pursuit_predictor.cpp](#srcpursuit_predictorcpp) — 6
- [src/reconnect_gate.cpp](#srcreconnect_gatecpp) — 7
- [src/separation.cpp](#srcseparationcpp) — 2
- [src/target_scheduler_node.cpp](#srctarget_scheduler_nodecpp) — 1
- [src/tree_detector_node.cpp](#srctree_detector_nodecpp) — 8
- [src/vantage_planner.cpp](#srcvantage_plannercpp) — 1
- [test/test_candidate_generator.cpp](#testtest_candidate_generatorcpp) — 1
- [test/test_cell_world.cpp](#testtest_cell_worldcpp) — 10
- [test/test_coordination.cpp](#testtest_coordinationcpp) — 16
- [test/test_cost_grid.cpp](#testtest_cost_gridcpp) — 3
- [test/test_exchange_drain.cpp](#testtest_exchange_draincpp) — 8
- [test/test_failed_goal_blacklist.cpp](#testtest_failed_goal_blacklistcpp) — 9
- [test/test_fov_evaluator.cpp](#testtest_fov_evaluatorcpp) — 8
- [test/test_global_allocator.cpp](#testtest_global_allocatorcpp) — 5
- [test/test_map_cache.cpp](#testtest_map_cachecpp) — 3
- [test/test_meeting_attendance.cpp](#testtest_meeting_attendancecpp) — 1
- [test/test_metrics_logger.cpp](#testtest_metrics_loggercpp) — 4
- [test/test_plan_map_query.cpp](#testtest_plan_map_querycpp) — 1
- [test/test_planner_util.cpp](#testtest_planner_utilcpp) — 11
- [test/test_pursuit_predictor.cpp](#testtest_pursuit_predictorcpp) — 7
- [test/test_reconnect_gate.cpp](#testtest_reconnect_gatecpp) — 9
- [test/test_scoring.cpp](#testtest_scoringcpp) — 1
- [test/test_separation.cpp](#testtest_separationcpp) — 4
- [test/test_team_model.cpp](#testtest_team_modelcpp) — 3
- [test/test_tree_detector.cpp](#testtest_tree_detectorcpp) — 8
- [test/test_vantage_planner.cpp](#testtest_vantage_plannercpp) — 5

## CMakeLists.txt

### cmake-git-rev-stamp

**Build-time git revision stamp** — in `Provenance`, attached to `set(EXPLO_PLANNER_GIT_REV_HDR` (line 26)

```text
Stamp the source revision into the planner binary so every experiment event
log can say which code produced it. No derived artefact in this project
carried provenance before, and the metrics CSV's schema changed silently
between campaigns with nothing in the data to date it.

Stamped at BUILD time, not configure time, into a generated header. This used
to be an execute_process() right here, which runs exactly once — when the
build tree is configured — so every later source edit rebuilt the binary
while leaving the old revision compiled into it. Campaign cr5 ran a binary
stamped 6ce7ad0 against a manifest recording 1a097dc for that reason, and the
gate's check 3b, whose whole job is comparing those two, could not pass on a
single cell of it.

The custom target re-runs the query on every build; the script writes the
header only when the answer changes, so the extra recompile happens on the
builds after a commit and not otherwise. Degrades to "unknown" outside a git
checkout rather than failing the build.
```

### cmake-test-exchange-drain

**Exchange-drain test runs the predicate** — in `Tests`, attached to `ament_add_gtest(test_exchange_drain test/test_exchange_drain.cpp)` (line 243)

```text
Gen-33 drain release, run rather than scanned: test-plan 7 and the
behavioural half of 8. The predicate was extracted from doReturnSync into
the lib for this; the node's wiring of it stays a scan in
test_gen21_latched_hold.
```

### cmake-test-endpoint-contract

**Endpoint test scope** — in `Tests`, attached to `ament_add_gtest(test_endpoint test/test_endpoint.cpp)` (line 266)

```text
T5. The endpoint contract: the three events that decide when a run is over.

It said "plus the departure arithmetic's edges" until 2026-09-18, and those
nine tests were deleted with RendezvousScheduler::shouldDepart in generation
19 — departure is no longer a `remaining <= need` comparison, so there are
no edges left to pin. See the GROUP A tombstone in test_endpoint.cpp, which
records what they asserted and which one hazard outlived them.

What remains is the logged rows (GROUP B) and two source scans of the node
(GROUPS C and E). The split from test_experiment_log stands on the first of
those alone: that file owns the writer, this one owns the ORDER and the
once-per-run-ness of what the writer emits, which is the property every
campaign's primary metric is a stamp on.
```

### cmake-test-endpoint-node-scan

**Why test_endpoint scans the node source** — in `Tests`, attached to `target_compile_definitions(test_endpoint PRIVATE` (line 281)

```text
The once-per-run mission-return guard lives in explo_planner_node.cpp, which
is NOT in explo_planner_lib's source list above, so nothing links it and no
test can call it. The guard is still the load-bearing half of F1, so
test_endpoint reads the node's SOURCE and asserts the latch is present and
ordered — the same technique, and the same justification, as
test_experiment_log reading the writer for kEventKinds. Source tree, not
install tree: the claim is about the code being built.
```

### cmake-test-gen20-rendezvous

**Rendezvous invariants source scan** — in `Tests`, attached to `ament_add_gtest(test_gen20_rendezvous test/test_gen20_rendezvous.cpp)` (line 306)

```text
T9. Generation 20's rendezvous invariants. Every one of them lives in the
node — the countdown's write-before-arm ordering, the five-site separation
predicate, the appointment-manoeuvre latch's keying — so this is the same
source-scan technique as test_endpoint's GROUP C, for the same reason and
with the same stated limits. It links the library only for gtest's sake; it
calls nothing from it.
```

### cmake-test-gen21-latched-hold

**Latched hold source scan** — in `Tests`, attached to `ament_add_gtest(test_gen21_latched_hold test/test_gen21_latched_hold.cpp)` (line 317)

```text
T10. Generation 21's at-the-rendezvous hold. Same source-scan technique as
T9 and for the same reason (the node is not in the library's source list),
but a separate target on purpose: three of four invariants are guards on
paths in functions a reader of the hold never opens, and each is individually
sufficient to reintroduce an asymmetric corruption of the primary endpoint.
```

### cmake-test-gen22-start-hold

**Pre-mission hold scans node and harness** — in `Tests`, attached to `ament_add_gtest(test_gen22_start_hold test/test_gen22_start_hold.cpp)` (line 327)

```text
T11. Generation 22's pre-mission hold. Same source-scan technique as T9 and
T10, and a separate target for the same reason: the hold is four edits in
four places plus one invariant that is not an edit at all (heartbeatTick's
handshake must keep running while a robot is held, or the hold is a 60 s
cost in all four arms that buys nothing).

THIS IS THE FIRST TEST THAT SCANS THE HARNESS, and the second compile
definition is why it has to. mission_start_hold_sec defaults ON in the node,
so "the harness forgot it" is not the risk; the risk is the harness passing
it in the ARM-CONDITIONAL `EXTRA` array rather than the unconditional
per-robot argv, which would apply the hold to rendezvous and hybrid only and
turn a startup cost into an apparent treatment effect in exactly the two
arms under test. That is a property of run_explo_sim_rviz.sh and of nothing
in the C++, so the C++ cannot be where it is checked.
```

### cmake-test-gen23-contagion

**Contagious arming: called and scanned halves** — in `Tests`, attached to `ament_add_gtest(test_gen23_contagion test/test_gen23_contagion.cpp)` (line 347)

```text
T12. Generation 23's contagious arming: one robot's broken view of the team
arms the whole team, in one hop, over a new TeamWorld field.

THE FIRST OF THESE THAT IS HALF EXECUTABLE, and the split is deliberate.
The wire field's storage semantics live in TeamModel, which IS in
explo_planner_lib's source list, so "the bit attaches to its sender",
"gossip never relays it" and "it has no TTL of its own" are CALLED rather
than scanned — and those are the three whose failure is silent, because a
relayed claim still produces a plausible-looking armed team. Everything
downstream (the five agreement sites, the producer's own-read-not-armed-
state rule, the finished-peer exemption) is in explo_planner_node.cpp,
which is not in that list, so it carries the same source-scan limit as
T9-T11. There is no glob in this file to blame for that — the list above is
explicit — but do not read the limit as one line away from lifting: the
node TU also defines main(), so a test that pulls any node symbol out of
the archive collides with gtest_main's main(). Making the node testable
means splitting main() off first, not editing this list.

A separate target from T9-T11 for the usual reason and one more: generations
18 and 19 were BOTH the five-site rule broken, in opposite directions, and
both reached a smoke run green. This target is the standing check that arm
and release read one predicate, and it must stay runnable against a mutated
node on its own.
```

## cmake/StampGitRev.cmake

### stamp-why-script-mode

**Why the stamp runs in script mode** — in `Top level`, attached to `if(NOT DEFINED OUT OR NOT DEFINED SRC)` (line 3)

```text
Run in script mode (`cmake -P`) from a custom target, so it re-evaluates on
every build. The obvious spelling — execute_process() at the top of
CMakeLists.txt — runs once, when the build tree is CONFIGURED, and never
again: editing a source file and rebuilding leaves the old revision compiled
into the binary. That is not a theoretical decay. Campaign cr5 ran a binary
stamped 6ce7ad0 while its manifest recorded 1a097dc, because the build tree
had been configured at P5 and every commit since had only rebuilt. The gate's
check 3b compares those two and could not pass on any cell.
```

### stamp-hash-not-describe

**A hash, not describe --tags** — in `Top level`, attached to `execute_process(` (line 21)

```text
A HASH, deliberately, and not `describe --tags`. Every consumer of this
string matches it against a hash: the harness manifest records
git_explo_planner from `git rev-parse --short`, gate check 3b asserts the
manifest startswith the baked rev's first seven characters, and
equiv_pair.sh requires exact equality with `rev-parse --short` to prove a
rebuild happened. The old spelling carried --tags, which is harmless only
while the repo has no tags: the first `git tag v1.0` would stamp binaries
"v1.0-dirty", check 3b would hard-fail every cell of the campaign running
at the time, and the cause would look nothing like the tag that caused it.
```

### stamp-dirty-marker

**The -dirty marker** — in `Top level`, attached to `execute_process(` (line 39)

```text
The "-dirty" marker, which rev-parse does not provide and gate check 3c
exists to read: a campaign whose binary was built from uncommitted work
is not reproducible from any commit. `status --porcelain` counts
untracked files too, matching equiv_pair.sh's dirt test — an untracked
source file survives `git checkout` and would compile into both sides of
a supposedly-controlled comparison.
```

## config/targets_flatforest.yaml

### targets-flatforest-trunks

**Choosing the flatforest target trunks** — in `Top level`, attached to `target_scheduler:` (line 4)

```text
Companion to config/targets.yaml (generic placeholders at (5,3)/(-4,-6),
which stand on empty ground in this world — exploitation there can only ever
close PARTIAL). Every centre below is a REAL trunk read straight out of the
world SDF that robot_sim.launch.py loads:
  hmr_sim/worlds/flatforest/flatforestv2.sdf  (world name "flatforest",
  80 "Oak tree*" models, ground_plane at z = 0, robot spawns at (0, 0, 0.5)).
The x/y are the models' own <pose> values, so they are exact, not measured
off a map.

Trunk geometry (measured off the model's collision/visual mesh,
~/.ignition/fuel/.../openrobotics/models/oak tree/7/meshes/oak_tree.dae,
whose node transform scales inches -> metres by 0.0254), radial extent about
the model origin — which is the target centre, and is NOT the trunk axis at
low z (the bark centroid sits ~0.4 m off it there):
  z 0.0-0.2 m (root flare / skirt)  bark radius <= 1.70 m  <- target_radius
  z 0.2-0.6 m (the sightline band)  bark radius <= 1.10 m
  z 0.6-1.2 m (clean trunk)         bark radius ~  0.40 m
  z 1.2-6.5 m                       canopy, out to ~4.7 m radius
`target_radius` is deliberately NOT the 0.4 m clean-trunk radius:
lineOfSightClear() stops the ray within (radius + one voxel) of the trunk
axis and treats anything inside that as the trunk itself, so under-declaring
it makes the tree's own root flare read as an occluder and fails the vantage.
It also sets the ring: standoff = radius + vantage_standoff_m (2.0).

1.80 m is set from a measured run, not from the mesh alone. The first pass
used 1.10 (the sightline-band extent) and 4 of the 5 vantages that were
actually driven and dwelled came back LoS BLOCKED, across two different
trunks and all three ring angles — target 1 closed PARTIAL 1/3, target 2
0/3. The map is voxelised at 0.10 m, so a ray at ground + 0.30 m samples the
cell spanning z 0.2-0.3 and can clip the *flare* band, not just the
sightline band; the declared radius has to cover the flare's 1.70 m. At 1.80
the ray stops 1.9 m from the axis, outside all of it, and the ring moves to
3.8 m — still well inside fov_max_range and under the canopy at ray height.

Why these three trunks:
  * isolated — nearest neighbouring tree is 9.6-11.0 m away, so the 3.8 m
    vantage ring never lands inside another trunk;
  * spread over three directions (NE / S / NW) and 13-19 m from spawn, well
    apart from each other (28-34 m), so the planner traverses between them
    and exploration resumes in between;
  * FAR from spawn on purpose. A vantage needs mapped ground under it
    (terrain-relative z); when a target activates with its ring unmapped the
    vantages are rejected `noground` and the planner falls back to marching
    at the trunk to map the area — which only works if there is somewhere to
    march FROM. Released next to its own ring, the planner has no approach
    waypoint left, cannot map the far side, and idles out its give-up timer:
    a 7 m target closed PARTIAL 1/3 that way with `valid=1 noground=2`. Every
    target here is far enough that the approach march maps its ring first.
  * "Oak tree_19" at (1.88, 2.01) is deliberately NOT used: MaleVisitorOnPhone
    stands 0.12 m from that trunk and would occlude a vantage, and its ring
    would sit on top of the spawn point.
The canopy is invisible to the navigator (simple_nav_3d counts obstacles only
between 0.40 and 1.00 m above ground), so driving the ring under the branches
is fine.

  id  SDF model      centre (x, y)     nearest tree   dist from spawn
  1   Oak tree_42    (  9.96,  8.34)     10.27 m         13.0 m
  2   Oak tree_4     (  4.40, -18.81)    11.01 m         19.3 m
  3   Oak tree_39    ( -9.85,  11.61)     9.60 m         15.2 m
```

## config/targets_map_test2.yaml

### targets-map-test2-trunks

**Choosing the map-test-2 trunks** — in `Top level`, attached to `target_scheduler:` (line 8)

```text
How these were chosen (the run is bag replay: NOTHING drives to the
vantages). At the time, exploitation was NOT terrain-adapted, so the trunks
were picked to suit the *absolute* z knobs. That constraint is gone —
exploitZAt() now snaps vantage z and the LoS sightline to the local ground —
but the pair still demonstrates the ring cleanly, so it stays as solved:
  * ground_z <= -1.4  — scovox projected the planning_map over the ABSOLUTE
    band [-1, 2]; on higher ground the ground plane itself landed in the band,
    marked every cell occupied and every vantage was rejected as not-free.
    (The runbook now also keeps the planning_map off entirely.)
  * sightline clearance — the LoS ray marched at the absolute
    candidate_robot_z (0.3), which had to sit ~2 m above the local ground and
    well under the canopy to actually strike the trunk.
  * open ring — with vantage_start_angle_deg: 30 the three vantages
    (30/150/270 deg, standoff = radius + 2.0 m) land in free, LoS-clear cells.
    Both the angle and these centres were solved together; changing one
    without re-checking the other will start rejecting vantages.

  id  centre (x, y)   ground_z  radius  closest approach   ring @ start=30
  1   ( 5.43, -8.10)   -1.70     0.25   4.9 m at t=151 s   2/3 free, 2/3 LoS
  2   (15.93, 16.57)   -2.65     0.27   5.4 m at t=219 s   3/3 free, 2/3 LoS
```

## include/explo_planner/candidate_generator.hpp

### cand-terrain-relative

**Terrain-relative candidate height** — in `CandidateConfig — declarations`, attached to `bool  terrain_relative     = false;` (line 30)

```text
Terrain-relative (3D) mode. When true AND a map is provided, each
candidate's z is set to the local ground elevation (MapCache::groundZAt,
searched in a window around the reference z: the robot z for polar
candidates, the centroid z for frontier candidates) plus z_clearance.
Columns with no detected ground fall back to the reference z unchanged.
When false (default) the legacy flat-world behaviour is preserved
bit-for-bit: every candidate sits at the fixed absolute robot_z.
```

### cand-roi-xy-box

**Candidate XY bounding box** — in `CandidateConfig — declarations`, attached to `float roi_min_x   = -1e9f;` (line 42)

```text
Hard XY bounding box on candidate positions (map frame, metres). The
generator drops any candidate (radial or frontier) whose centre falls
outside [roi_min_x, roi_max_x] x [roi_min_y, roi_max_y]. Set the dscovox
planning_map size + origin to match this box so the global planner is
constrained to the same area.
```

### cand-roi-z-clamp

**Why terrain-snapped z is clamped** — in `CandidateConfig — declarations`, attached to `float roi_min_z   = -1e9f;` (line 51)

```text
Vertical band a terrain-snapped candidate z is clamped into (map frame,
metres, ABSOLUTE — the node pushes the effective robot-relative band each
PLAN tick via setRoiZ). This is the band MapCache actually ingested, so a
candidate outside it is a viewpoint whose FOV origin sits in space the map
holds nothing for: every ray from there walks un-ingested cells and scores
them as the Beta(1,1) prior, which is maximal — the planner would chase
its own blind spot. Reachable in terrain mode because ground search is
referenced to the CENTROID's z for frontier candidates, so ground +
z_clearance can land above the band. Defaults are wide enough to be inert
in flat mode, where candidates sit at the fixed absolute robot_z.
```

### cand-frontier-candidates

**Frontier candidate height** — in `CandidateGenerator — declarations`, attached to `void addFrontierCandidates(` (line 78)

```text
Inject frontier centroids as additional candidates. In flat mode the
centroid z is flattened to robot_z; in terrain-relative mode (with a
map) the candidate is snapped to ground + z_clearance, searched around
the centroid's own z, falling back to the centroid z when no ground is
found (frontiers naturally border unobserved columns).
```

## include/explo_planner/cost_grid.hpp

### costgrid-build-unknown-traversable

**Why unknown cells are traversable** — in `CostGrid — declarations`, attached to `void build(const nav_msgs::msg::OccupancyGrid& planning_map,` (line 34)

```text
Resample the cost grid from a fresh planning_map.

`obstacle_threshold` matches the planner's existing isCellFree threshold.
A cell is impassable iff it is KNOWN and at or above the threshold, i.e.
`v >= 0 && v >= obstacle_threshold` (`cost_grid.cpp:61-66`).

UNKNOWN (-1) IS TRAVERSABLE. This is deliberate and is the opposite of
what this comment used to claim. The flood's job is to find free pockets
sealed off by *known* obstacles; it is not a free-space test. Blocking
unknown would make every frontier a wall and the reachability filter
would reject exactly the candidates exploration exists to reach. The
per-candidate `isCellFree()` check in the node is what refuses to stand
a robot on an unknown cell — the two filters have different jobs and
must not be "made consistent".
Origin / resolution / dims are captured from the OccupancyGrid so the
caller can convert candidate world XY to grid coords without keeping a
pointer to the original message.

Calling build(...) clears any prior flood result.
```

### costgrid-flood-radius-cap

**The flood radius cap** — in `CostGrid — declarations`, attached to `void floodFrom(const Eigen::Vector3f& source_xy, float radius_cap_m);` (line 56)

```text
Run a single-source bounded flood from `source_xy`. Cells whose
shortest-path distance from the source exceeds `radius_cap_m` are not
touched and keep the sentinel kInfCost.

Setting `radius_cap_m <= 0` (or NaN) runs a genuinely unbounded flood —
the cap becomes +inf. THE GRID DIAGONAL IS NOT AN EQUIVALENT BOUND: these
are *walked* Dijkstra distances, which routinely exceed the straight-line
diagonal around serpentine corridors and U-shaped obstacles, so a cap set
at the diagonal still leaves reachable cells at kInfCost and callers still
read them as unreachable. That was a real defect; see the note at the cap
computation in cost_grid.cpp. The bound is what makes this <1 ms per PLAN
tick on the live system.

Diagonal moves cost sqrt(2) * resolution; orthogonal cost 1 * resolution.
Out-of-bounds source returns cleanly: every cell stays at kInfCost.

Calling floodFrom(...) again replaces the previous flood result without
rebuilding the obstacle layer.
```

### costgrid-blocked-layer

**The obstacle layer and malformed maps** — in `CostGrid — declarations`, attached to `std::vector<uint8_t> blocked_;` (line 110)

```text
Obstacle layer sampled from the OccupancyGrid at build() time.
blocked_[gy * dims_x_ + gx] == true for cells that the flood will not
enter: KNOWN cells at or above `obstacle_threshold` (occupied or
inflated). Unknown (-1) cells are NOT blocked — see build() above.
The one exception is a malformed message whose `data.size()` disagrees
with `dims_x_ * dims_y_`, where every cell is marked blocked so the
flood is empty and the caller degrades to "nothing reachable" instead
of indexing past the end of the array.
```

## include/explo_planner/exchange_drain.hpp

### drain-tumbling-window

**The tumbling drain window** — in `DrainWindow — declarations`, attached to `struct DrainWindow {` (line 30)

```text
A TUMBLING WINDOW, ON THE SETTLE'S CLOCK. `start_sec` is the start of the
window currently being measured, as an offset into the settle (one hold,
one epoch, nothing extra to keep in step); -1 while no window is open.
`base` is the fusion-counter vector as it read when the window opened; the
rate is the difference over the window divided by its length, which is why
a window is closed and re-opened rather than slid — a ring buffer would buy
sub-W granularity on a decision whose whole point is that it is taken at
the end of a quiet interval.

IT IS ALSO THE HARD FLOOR. No window has elapsed before W seconds, so no
drain release can happen before then, without a second mechanism saying so.

Reset to a default-constructed DrainWindow wherever the hold is cleared, so
the next visit starts with no window open.
```

## include/explo_planner/failed_goal_blacklist.hpp

### failed-goal-site-expired

**Expired sites keep their count** — in `FailedGoalSite — declarations`, attached to `struct FailedGoalSite {` (line 12)

```text
One clustered site: a position, when it last failed, and how often.

`expired` separates "this site no longer suppresses goals" from "this site
is forgotten". A site past its TTL stops vetoing candidates but keeps its
failure count, because the count is what retirement is counted in — see
prune().
```

### failed-goal-blacklist-design

**Failed-goal blacklist modes and retirement** — in `FailedGoalBlacklist — declarations`, attached to `class FailedGoalBlacklist {` (line 26)

```text
Records goal positions the robot failed to reach (navigate timeout / no
progress) so the planner can skip re-picking the same unreachable target
while it's still "hot". Without this, a physically stuck robot keeps the
same pose -> same scores -> re-picks the same goal forever.

Time is passed in as seconds (the caller's clock) rather than read
internally, so the prune/expiry logic is deterministic and unit-testable
without a ROS clock.

Two modes, selected per call site:
  - cluster_radius_m <= 0 (the default): every add() creates a new record.
    This is the historical append-only behaviour and is what `visited_goals_`
    uses; nothing about it changes.
  - cluster_radius_m > 0: an add() within that radius of an existing record
    increments that record's failure count instead of appending. This is
    what turns a bag of points into a per-SITE failure history, which is
    what retirement needs. The record keeps its first-seen centre so the
    suppression disc does not drift across repeated failures.

Retirement (opt-in via setRetireAfter) exists because TTL expiry alone
cannot stop a permanent terrain trap: the entry ages out while the robot is
busy failing somewhere else, and the same unreachable goal wins the argmax
again. A site that has failed `retire_after` times is held for the rest of
the run and is only released by actually reaching it (clearNear).

For that to be more than a comment, the failure COUNT has to outlive the
suppression window. It did not: prune() erased the whole record, so a
re-failure at the same place started again at 1, and retirement could only
ever fire when every failure landed inside one TTL. That is the opposite of
the case it was written for. Measured over at1+sr3 — 64 robot-runs, 177 nav
failures — the count was 1 every single time and retirement never fired
once. So an expired site is now kept as HISTORY (see prune): it stops
suppressing, but add() can still find it and carry the count forward.
```

### failed-goal-prune-expiry

**Pruning erases or expires** — in `FailedGoalBlacklist — declarations`, attached to `void prune(double now_sec, double ttl_sec);` (line 71)

```text
Age out entries older than `ttl_sec` relative to `now_sec`. Retired sites
are never aged out.

With retirement ON the entry is marked expired rather than erased: it
stops suppressing goals immediately, but its failure count is kept so a
later failure at the same place counts as the second, not the first.
With retirement OFF (the default, and what `visited_goals_` uses) the
entry is erased exactly as before — there is no count to preserve.
```

### failed-goal-clear-near

**Reaching a goal clears its history** — in `FailedGoalBlacklist — declarations`, attached to `std::size_t clearNear(const Eigen::Vector3f& pos, double radius_m);` (line 92)

```text
Forget every entry within `radius_m` of `pos` — retired and expired
history included. Returns how many were removed. Arriving at a goal is
proof the ground is reachable, which outranks any amount of failure
history, so the carried-over count is dropped here too and the next
failure there genuinely starts at 1.
```

### failed-goal-amnesty-order

**Amnesty ordering as a free function** — in `amnestyOrderBefore`, attached to `bool amnestyOrderBefore(const FailedGoalBlacklist& bl,` (line 115)

```text
Strict weak ordering for the planner's amnesty valve: true if suppressed
candidate `a` should be re-attempted before `b`. Non-retired first, then
least-recently-failed.

A free function rather than a lambda inside the planner because this
ordering fails silently: get it wrong and the planner still returns a goal
on every call, just the worst available one, with nothing in the logs to
distinguish that from the intended pick. Out here it has a known-answer
test. Rationale for the partition itself is at the definition.
```

## include/explo_planner/fleet_identity.hpp

### fleet-id-team-size-cap

**Team size policy cap** — in `File scope`, attached to `inline constexpr int kMaxTeamSize = 8;` (line 41)

```text
Policy cap on team size. A POLICY cap, not a representation one: the masks
are uint32 and would hold 32. Eight is the largest fleet any of the
data structures here have been reasoned about at, and a cap that is checked
beats a cap that is assumed — an unnoticed 33rd robot would shift bits off
the end of every mask and corrupt the knowledge gate silently.
```

### fleet-id-fleet-mask

**Mask naming the whole fleet** — in `fleetMask`, attached to `inline uint32_t fleetMask(int size) {` (line 55)

```text
Mask naming every robot of a fleet of `size`, self included. Empty for a
size outside the policy cap, so a mask built from an unvalidated size is
empty — and therefore matches nothing — rather than naming robots that do
not exist. `(mask & fleetMask(n)) == fleetMask(n)` is the test for "this
robot reported direct contact with the WHOLE team", which is what makes a
team-completeness question answerable from a peer's broadcast mask instead
of only from our own links.
```

### fleet-id-names-hash

**Portable hash of the fleet names** — in `teamNamesHash`, attached to `uint32_t teamNamesHash(const std::vector<std::string>& names);` (line 76)

```text
FNV-1a 32-bit over the names joined by NUL, in order.

Hand-rolled rather than std::hash because this value goes ON THE WIRE and
is compared between processes: std::hash is implementation-defined and is
permitted to differ between two libstdc++ versions, let alone two
standard libraries. A config check whose answer depends on which machine
compiled the binary is worse than no check — it would report a config
mismatch on a correctly configured fleet, and the fix people would reach
for is deleting the check.

Order matters (it is what defines the ids), so this is deliberately NOT
invariant to permutation: two robots that agree on the membership but not
the order do not agree on identity.
```

### fleet-id-error-field

**Rejected versus absent fleet param** — in `FleetIdentity — declarations`, attached to `std::string error;` (line 102)

```text
Human-readable reason the param was rejected; empty when the param was
accepted OR when it was absent. Distinguishing those two is the caller's
job and is what `configured` plus emptiness of `names` is for: an absent
param is legacy operation, a malformed one is a config error the node
must refuse to run past.
```

### fleet-id-validation-rules

**Validation rules for team_robot_names** — in `makeFleetIdentity`, attached to `FleetIdentity makeFleetIdentity(const std::vector<std::string>& names,` (line 120)

```text
Resolve `team_robot_names` for the robot called `self_name`.

Empty `names` -> unconfigured, no error (the default, legacy path).
Otherwise the array must be a well-formed fleet definition or the result
carries `error` and `configured == false`:
  - at most kMaxTeamSize entries;
  - no empty entry (an empty name cannot be matched against `robot_name`,
    and would give a bit nobody can claim);
  - no duplicates (two robots would share one bit, and the knowledge gate
    would credit one with the other's observations);
  - `self_name` must appear (a robot that is not in its own fleet cannot
    set its own bit, so every mask it publishes would be a lie).
```

## include/explo_planner/fov_evaluator.hpp

### fov-config-test-placeholders

**FovConfig defaults are test placeholders** — in `FovConfig — declarations`, attached to `struct FovConfig {` (line 14)

```text
NOT a sensor model. Every field below is overwritten from parameters by
explo_planner_node before the evaluator is constructed, so these values
never reach a run; the deployed sensor lives in `shared_params.yaml`
(`fov_*`) and is traceable to the SDF. They are deliberately left at the
old narrow-camera numbers so that the unit tests, which construct a
FovConfig and set only the fields they are exercising, keep testing the
directional geometry they were written for. Do not "correct" them to the
deployed values: that would create a second place for the sensor model to
drift, and would silently change what ten tests assert.
```

### fov-roi-z-band

**ROI clip must match map band** — in `FovConfig — declarations`, attached to `float roi_min_x  = -1e9f;` (line 32)

```text
XYZ ROI bounds.  Rays are clipped at the ROI boundary so the
evaluator never scores voxels outside the region of interest. The z
band must match the volume the local map_cache_ actually holds (in
dscovox mode that is the GetRegion fetch band): otherwise rays leaving
the band traverse absent voxels and score them as the Beta(1,1) prior,
inflating info gain for upward-pointing rays into unfetched space.
```

### fov-set-roi-z

**Re-banding the ray clip** — in `FovEvaluator — declarations`, attached to `void setRoiZ(float roi_min_z, float roi_max_z) {` (line 57)

```text
Re-band the vertical ray clip. Terrain-relative mode moves the map z-slab
with the robot each PLAN tick; the ray clip must track it so out-of-band
space stays "empty" (no score, no occlusion) rather than reading absent
voxels as the max-uncertainty prior. Ray directions only depend on the
FOV so no recompute is needed.
```

## include/explo_planner/home_trail.hpp

### home-trail-nearest-crumb

**Why retrace never selects crumb 0** — in `nearestCrumbFromOne`, attached to `inline int nearestCrumbFromOne(const std::vector<Eigen::Vector3f>& trail,` (line 31)

```text
Index of the crumb nearest `pos`, searching from 1 (never home itself).
Returns 1 for a trail with a single usable crumb; the caller must not call
this with `trail.size() <= 1`, where there is no retrace to engage at all.

Selecting crumb 0 would put the planner in RETRACE while making the
republished goal, and the approach metric, identical to DIRECT: the robot is
re-sent the very goal that just failed twice, under a label saying
otherwise. It is also a one-way door — the escape branch keys off
`home_mode_ == RETRACE`, so a nominal retrace that never retraces sends
every later fire straight to an escape and deletes three rungs of the
ladder.
```

### home-trail-remaining-distance

**Remaining distance along the trail** — in `remainingTrailDistance`, attached to `inline float remainingTrailDistance(const std::vector<Eigen::Vector3f>& trail,` (line 53)

```text
Distance from `pos` to crumb `idx`, plus the trail from `idx` down to home.
This is what the robot still has to drive while retracing, and it is the one
definition of "remaining" that both the watchdog and the nav budget use.

No trail[0]-to-home term: they are the same point (see the file comment), so
writing it out would only suggest a correction that is not happening.
`idx` outside [1, size) is a spent or unset trail and yields 0.
```

### home-trail-escape-band-crumb

**Choosing the escape crumb** — in `pickBandCrumb`, attached to `inline int pickBandCrumb(const std::vector<Eigen::Vector3f>& trail,` (line 70)

```text
Nearest crumb whose distance from `pos` falls inside [band_min, band_max],
skipping `exclude_idx` (the crumb a previous escape already used) and
searching from 1. Ties break toward the HIGHER index — the most recently
visited crumb, which is the one behind the robot. -1 when nothing qualifies,
which is the caller's signal to use the rotate-behind fallback.

From index 1 for a sharper reason than the retrace case: for a robot stalled
1.5-6 m out, home sits squarely in the escape band, so crumb 0 would aim the
escape at the one goal already known to have trapped it — and AHEAD of it,
not behind, so the ~180 deg rotate-in-place the mechanism depends on never
happens. An escape onto home is a resend wearing an escape's label.
```

## include/explo_planner/map_cache.hpp

### map-cache-roi-rebuild

**ROI-clipped rebuild and bad resolution** — in `MapCache — declarations`, attached to `bool updateFromScovoxMap(const scovox_msgs::msg::ScovoxMap& msg,` (line 27)

```text
Rebuild grid from a ScovoxMap, keeping only voxels whose position lies in
the inclusive AABB [roi_min, roi_max]. The fused-map topic carries the
whole map; the planner re-applies its ROI clip here so map_cache_ stays
bounded to the ROI as the old per-region GetRegion service made it (frontier
extraction and map-stats both walk the whole grid, so the clip matters).
Non-finite positions are dropped. NB: this clips on voxel position; the old
service clipped in coord space, so results match for resolution-aligned ROI
bounds and may differ by one voxel layer at a non-aligned min boundary.

@return true if the grid was rebuilt. false means msg.resolution was
        positive but not finite (+inf), which would make Bonxai's
        inv_resolution zero and every posToCoord a float->int32 cast of a
        non-finite value — UB that surfaces as garbage coordinates, not a
        crash. Unlike the constructor and updateFromLogOddsCloud, which
        throw on a bad resolution because that is a config error, this
        value arrives over the wire mid-run: the previous grid is kept and
        the message is dropped, so one malformed publish cannot take the
        node down in the field. Callers should log it (throttled) and
        treat it as "no new map this tick".
```

### map-cache-ground-z

**Ground elevation from the voxel column** — in `MapCache — declarations`, attached to `float groundZAt(float x, float y, float z_low, float z_high,` (line 72)

```text
Estimate the ground elevation at world (x, y): the z of the top FACE of
the ground voxel stack in that column. The search scans the column from
z_low upward to z_high; the FIRST (lowest) occupied voxel
(p_occ >= occ_thresh) anchors the ground, then the walk continues up
through contiguous occupied voxels for at most stack_max_m (absorbs the
residual vertical measurement smear without climbing walls/trunks) and
the top face of the last stack voxel is returned. Lowest-first anchoring
makes the estimate robust to canopy/overhangs higher in the column.
Returns NaN when the window contains no occupied voxel (unobserved or
free-only column).
```

### map-cache-stats

**Whole-grid map statistics** — in `MapCache — declarations`, attached to `struct MapStats {` (line 85)

```text
Aggregate per-voxel statistics over the whole (already ROI-clipped) grid:
mean expected-information-gain, mean entropy, mean Beta variance, and the
frontier-voxel count (free voxels with >=1 unknown 6-neighbour). Walks
every active cell once. Extracted from the planner's LOG_STEP so it is
unit-testable and shares the frontier-neighbour logic with
findFrontierCentroids.
```

### map-cache-unknown-columns

**Unknown column fraction as area coverage** — in `MapCache — declarations`, attached to `double unknownColumnFraction(float min_x, float max_x,` (line 100)

```text
Fraction of x/y columns inside [min_x, max_x] x [min_y, max_y] that
contain NO observed voxel — a 2.5D "area coverage" measure over the
(already z-clipped) fused 3D map. A column counts as observed when any
grid cell (free OR occupied) projects into it, so trunk columns and
swept free space both count as covered; only never-seen ground-plane
cells raise the fraction. This deliberately ignores volumetric unknowns
(trunk interiors, canopy shadow are never observed and would put a
permanent floor under a 3D unknown fraction), matching the semantics of
the 2D planning_map unknown fraction used for coverage termination.
Bounds are mapped to coord space with the same floor() convention as
voxel ingest. Returns a value in [0, 1]; an empty grid gives 1.0.
Returns -1.0 on a degenerate box (max <= min) or non-finite bounds —
the caller's "cannot measure" convention.
```

## include/explo_planner/meeting_attendance.hpp

### meeting-announced-mode

**The announced mode and the DONE latch** — in `announcedMode`, attached to `uint8_t announcedMode(bool finished_announced, bool keeping_appointment,` (line 52)

```text
The level this robot publishes in TeamWorld/mode.

  finished_announced  the publisher's `finished` latch (coverage latched,
                      or reached State::DONE)
  keeping_appointment it is in a manoeuvre started to keep an appointment
                      (appointment_manoeuvre_) — driving to the agreed
                      cell, or standing at its barrier
  homing_announced    the publisher's homing latch
  done_announced      the DONE latch, owned by the caller; set here

DONE IS LATCHED, like every level on this field: max-merge cannot go back
down, so a level the robot later contradicts would be a permanent lie on
every peer. Once finished and not keeping an appointment, the robot is DONE
for the rest of the run, even if a later tick reads keeping again.

A FINISHED ROBOT KEEPING ITS APPOINTMENT READS EXPLORING, which on this
field means "no evidence it is leaving" (TeamWorld.msg/robot_mode), not
"confirmed exploring". Whether its run is over is `finished`'s question,
and `finished` still says yes.
```

### meeting-finished-peer-coming

**The finished peer still coming** — in `finishedPeerStillComing`, attached to `int finishedPeerStillComing(const TeamModel& team, int self_id);` (line 74)

```text
The first peer, by fleet id, that has finished exploring, has not said it
is leaving (mode below HOMING), and that this robot cannot currently hear —
not direct, not heard one way, not in the comms closure. -1 if none.

A PEER THIS ROBOT CAN HEAR IS NOT THIS FUNCTION'S CASE. Its mode reaches us
fresh, it is accounted for by a radio channel, and the barrier's existing
terms decide it — including the still-driving veto, which covers a finished
robot walking in while in range. What this adds is the peer whose last word
was "finished, coming", heard once and then lost to the radio.

An UNFINISHED absent peer is not this function's case either: the barrier
already waits for it (the unbounded appointment vigil).
```

## include/explo_planner/proximity_guard.hpp

### prox-escape-grace

**Post-escape immunity from new holds** — in `Config — declarations`, attached to `float escape_grace_sec = 30.0f;` (line 71)

```text
After the caller's max-hold escape hatch fires, the escaped-from peer
cannot START a new hold for this long — long enough to drive out of
the trigger disc. 0 disables (restores the pre-hatch wedge). The
immunity ends early if the peer moves (a moving peer is alive, and
alive peers get full yields).
```

### prox-decision-note

**What a hold decision reports** — in `ProximityGuard — declarations`, attached to `struct Decision {` (line 79)

```text
One evaluation result. When hold is true, peer_id/dist_m name the
nearest offending peer (for logging; the hold itself is boolean). When
hold is false they name the nearest outranking peer considered — if any
— and note says why it did not hold: "clear" (beyond the trigger),
"parked", "stale", or "no-peer" (nothing outranking is tracked), so a
release log can tell cleared-off from parked from comms-lost.
```

### prox-peer-pose-ingest

**Recording peer poses** — in `ProximityGuard — declarations`, attached to `void onPeerPose(const std::string& peer_id, const Eigen::Vector3f& pos,` (line 97)

```text
Record a peer pose observation, stamped with the LOCAL receipt time
(the caller's node clock). Sources: RobotIntent.robot_pos (1 Hz
heartbeat) and any configured peer localiser topics (~10 Hz); both feed
the same per-peer track, latest-wins. Self-echoes and non-finite
positions (a diverged localiser) are dropped.
```

## include/explo_planner/pursuit_predictor.hpp

### pursuit-dwell-sec

**Peer dwell time per cell** — in `Config — declarations`, attached to `double dwell_sec = 45.0;` (line 116)

```text
Time the peer spends WORKING a cell before moving on, seconds. This is
the I transition's whole content, and it dominates: at the 10 m cell
every campaign runs (CELL_SIZE_M:-10.0) and the 0.40 m/s the harness
passes for pursuit, the drive between two adjacent cells is ~25 s while
clearing one takes as long as the local planner needs — so a 45 s dwell
is already the larger term, and diagonally it is comparable rather than
smaller. (This read "at a 20 m cell and 0.5 m/s ... ~40 s" until
2026-09-18: no campaign has ever run a 20 m cell, and the two errors
happened to cancel into a plausible number.) Setting it to zero turns
the model into pure translation and will predict the peer far ahead of
where it is.
```

### pursuit-step-sec

**Chain step and horizon quantisation** — in `Config — declarations`, attached to `double step_sec = 5.0;` (line 129)

```text
Chain step, seconds. Smaller is a finer distribution and more work;
the cost is linear in (horizon / step) x tour length. It also quantises
the horizon: a candidate is scored at floor(horizon / step) steps, so a
step comparable to the drive between two cells makes the intercept
insensitive to the very thing it is trying to resolve.
```

### pursuit-offroute-half-life

**Off-route hazard half-life** — in `Config — declarations`, attached to `double offroute_half_life_sec = 180.0;` (line 136)

```text
Half-life of staying on the tour at all, seconds. After this long, half
the probability mass has drained into O. This is the parameter that
decides how much staleness is too much, and it is a half-life rather
than a cutoff so that the answer degrades continuously — a chase does
not become worthless one second after a threshold.

<= 0 disables the hazard: the peer never leaves the tour. That is the
honest reading of "no half-life", and the opposite of the one a
clamp-to-epsilon would give (an infinitely SHORT half-life, i.e. total
drain in one step) — which is a setting nobody would ask for by typing
zero. It exists so the chain's arithmetic can be checked against an
exact binomial; in a run it is a way to say "trust the tour".
```

### pursuit-max-horizon

**Prediction horizon bound** — in `Config — declarations`, attached to `double max_horizon_sec = 600.0;` (line 150)

```text
Bounds the PREDICTION HORIZON, seconds — `peer.age_sec + my drive`, not
my drive alone. The horizon is measured from the peer's last fix, so a
stale fix has already spent part of the budget before I move; scoring
against a report that old is the thing the bound exists to refuse.
Candidates over it are dropped rather than scored at a clamped horizon,
because scoring at the clamp would rank a cell I cannot reach inside the
prediction against cells I can, using a distribution that does not
describe the moment I would arrive. See pursuit_predictor.cpp's own note
at the rejection site: the age-inclusive form is the intended reading.
```

### pursuit-step-conservation

**Chain step conserves probability** — in `PursuitPredictor — declarations`, attached to `static void step(const CellWorld& world, const std::vector<int>& tour,` (line 198)

```text
Advance the chain one step, in place.

`p` is index-aligned with `tour` and `p_off` is the absorbed mass; the sum
of the two is invariant. Exposed for the conservation test and for
callers that want the distribution itself — see the header on why that
invariant is tested at the step and not only at the end.
```

## include/explo_planner/reconnect_gate.hpp

### reconnect-gate-unassigned-dead

**unassigned is a dead column** — in `GateVerdict — declarations`, attached to `int unassigned = 0;` (line 93)

```text
Cells no vehicle could legally take under the no-comms mask.

STRUCTURALLY ZERO, AND THEREFORE A DEAD COLUMN. It was meant to report
comms-mask starvation, and it cannot: the deciding robot is always built
in_comms, the mask exempts an in_comms vehicle from the known-by test
entirely, and an exempt vehicle can absorb any cell. So it is not merely
"expected to be 0" (which this said until 2026-09-18, alongside a reading
of what a nonzero value would mean) — nonzero is unreachable while self is
in the vehicle set, which it always is. Kept only so the column does not
change meaning mid-campaign; listed with the other dead columns in
CODE_TODO.
```

### reconnect-gate-evaluate

**Evaluating the gate** — in `evaluateReconnectGate`, attached to `GateVerdict evaluateReconnectGate(const CellWorld& world,` (line 111)

```text
Evaluate the gate.

`robots` is the full vehicle set as doPlan would build it for the allocator,
including the missing peers themselves — the value gate needs them present
in both solves, differing only in whether they are reachable. `missing` is
the set of peers the trigger is considering, i.e. the CANDIDATES; the gate
picks exactly one of them — the nearest it can locate — and prices and
values a manoeuvre that fetches that one. It is not a set of peers that all
get fetched, and at N >= 3 the distinction is the difference between a
correct verdict and a one-directional bias towards dispatch.

`cfg` supplies the allocator's polish/candidate limits; its `comms_mask`
field is IGNORED. Both solves run with the mask ON (2026-09-18) and differ
only in the vehicle set: in the "stay apart" solve every missing candidate
is out of comms, and in the "reconnect" solve the one chosen peer is back.
Switching the flag off for the second solve — which is what this used to do
— unmasks the whole fleet at once and cannot express a single reconnection.
```

### reconnect-gate-unshared-count

**The knowledge gate count** — in `unsharedCellCount`, attached to `int unsharedCellCount(const CellWorld& world,` (line 133)

```text
The knowledge gate on its own: how many non-UNSEEN cells are missing from at
least one unfinished peer's `known_by`. Exposed separately because it is the
half with a documented failure mode in both directions — vacuous-true if the
reset semantics regress, vacuous-false if the mask is ever widened by a
merge that did not actually transfer the status — and a count is testable
where a bool is only ever anecdote.
```

## include/explo_planner/separation.hpp

### separation-radius-bounds

**Separation radius bounds** — in `Config — declarations`, attached to `double radius_m = 20.0;` (line 81)

```text
Distance at which the discount has fully decayed, metres. Default 20,
the radius the correlation is stated at. Bounded to [1e-3, 1e6] by
configure(): the term computes d / radius in FLOAT, where a radius
below about 1e-38 rounds to zero and a candidate sitting exactly on a
teammate then evaluates 0/0 and logs a NaN discount, while a radius
above about 3e38 rounds to infinity and flattens the discount to a
constant across every candidate. Neither is a plausible request; both
are refused rather than clamped, for the reason configure() gives.
```

### separation-max-age

**Separation peer position max age** — in `Config — declarations`, attached to `double max_age_sec = 10.0;` (line 91)

```text
How old a peer position may be and still repel, seconds. NOT a
convenience bound — a separation term driven by a stale position pushes
this robot away from where the teammate WAS, which after a long outage
is uncorrelated with where it is and may be the exact ground it has
since left uncovered.

The default (10 s) is two TeamWorld heartbeats plus margin. It is
deliberately short, and under the 2026-09-03 radio that short bound is
self-consistent rather than restrictive: with a 30 m range horizon a
teammate is only HEARD while it is close, which is exactly when this
term should be acting. When the link is down the peer is beyond the
horizon, further away than `radius_m` anyway, and the term correctly
goes quiet instead of guessing.
```

### separation-anchor-eligibility

**Anchor eligibility is the caller's job** — in `SeparationTerm — declarations`, attached to `struct Anchor {` (line 107)

```text
One teammate this robot is willing to be repelled by, in map-frame XY.

ELIGIBILITY IS THE CALLER'S JOB and there are three parts to it, none of
which this unit can see: the position must exist, it must be fresher than
max_age_sec measured as a POSITION age (not as "when did we last hear
anything about this robot" — a relayed status update refreshes the second
while leaving the first untouched), and the peer must not have finished.
A finished teammate is parked and covering nothing, so being repelled by
it is pure loss.
```

### separation-configure-refuses

**Separation configure refuses, never clamps** — in `SeparationTerm — declarations`, attached to `std::string configure(const Config& cfg);` (line 123)

```text
Install a configuration. Returns "" when the term is usable as given, or
a human-readable reason it was DISABLED. Out-of-range values disable
rather than clamp: `weight = 3` and `radius_m = -20` are not weak
requests for separation, they are typos, and a term that quietly ran at
some clamped value would put a number in the manifest that the binary did
not use. Silence is the failure mode this project keeps paying for, so
the refusal is returned for the caller to log.

The one exception is `weight == 0`, which is OFF by request and returns
"" — it is the default and not an error. Note that `radius_m` and
`max_age_sec` are still validated in that case and still kept: they are
what the caller's separation DIAGNOSTICS are measured on, in every arm
including the untreated one.
```

### separation-eligible-anchor

**Why eligibleAnchor lives in the unit** — in `SeparationTerm — declarations`, attached to `bool eligibleAnchor(double position_age_sec) const;` (line 147)

```text
Whether a teammate with this position age may be used as an anchor.

This lives here rather than in the caller for one reason: `max_age_sec`
is otherwise a knob that is parsed, range-checked, written into the run
manifest and then consumed entirely outside anything a unit test can
reach. That is the precise shape of a failure this project has already
paid for more than once — a value that the manifest swears the run used
and the binary never read — and a review of this very file found that
replacing the age comparison with a constant would pass the whole suite.
Moving the comparison into the unit makes the knob testable.

`position_age_sec` is a POSITION age (see Anchor), and a negative value
means no position is held at all. Independent of `enabled()`: the
eligible-peer count is logged in every arm, treated or not.
```

### separation-apply-guard

**Apply scales only positive utility** — in `SeparationTerm — declarations`, attached to `static float apply(float utility, float discount);` (line 176)

```text
Apply a multiplier to a utility, safely.

Only a FINITE, STRICTLY POSITIVE utility is scaled, and both halves of
that guard are load-bearing:

  - Unreachable candidates carry U = -inf so they sort to the bottom.
    -inf * 0 is NaN, and a NaN score is a candidate whose position in the
    sort depends on which comparisons the sort happens to make. The sort
    is NaN-safe, but "sorts somewhere defined" is not the same as "is not
    produced", and a NaN here would be produced by exactly the
    configuration an experiment is most likely to run (weight = 1).

  - A negative utility scaled by a factor below 1 gets LARGER, so a
    discount would become a reward. Nothing currently produces one, which
    is precisely why it would go unnoticed if something started to.
```

## include/explo_planner/target_queue.hpp

### target-visited-vantages

**Visited vantages tracked by proximity** — in `Target — declarations`, attached to `std::vector<Eigen::Vector3f> visited_vantages;` (line 35)

```text
XY positions of vantages already dwelled at for this target — local
dwells plus canonical ring positions merged from peers. Vantages are
re-generated every plan tick, so "already visited" is tracked by proximity
to these recorded positions (see isVantageVisited) rather than by a
transient per-tick index. Z is ignored in the comparison.
```

### target-clear-mask

**The clear-LoS vantage index mask** — in `Target — declarations`, attached to `uint32_t clear_mask = 0;` (line 47)

```text
Bitmask of vantage indices dwelled with clear LoS (bit i = ring index i).
The ring is deterministic from (center, radius), so indices are globally
meaningful across robots; this is what gets broadcast in RobotIntent and
what makes local + peer credit idempotent to merge (an index counts once
no matter how many robots dwell it or how often the mask is re-received).
Indices >= 32 fall back to the plain counter (no team sharing).
```

### target-queue-deactivate

**Deactivate stands exploitation down** — in `TargetQueue — declarations`, attached to `void deactivate();` (line 84)

```text
Demote the ACTIVE target back to PENDING and clear the active slot.
Unlike markActiveDone() this does NOT close the target — it stands
exploitation down so a higher-priority behaviour (the rendezvous barrier)
can run without the target being lost or its exploit claim left live.
The next activate() picks the same target up again. No-op if none.
```

### target-record-vantage-dwell

**Recording a vantage dwell** — in `TargetQueue — declarations`, attached to `void recordVantageDwell(const Eigen::Vector3f& vantage_xy, bool los_clear,` (line 91)

```text
Record that the robot dwelled at a vantage of the ACTIVE target.
`vantage_index` is the deterministic ring index of the vantage (< 32 for
team credit sharing; -1 / out-of-range falls back to counter-only).
A clear-LoS dwell on an index already credited (e.g. merged from a peer
that dwelled it first) does not double-count.
```

### target-merge-peer-dwells

**Merging a peer's dwell mask** — in `TargetQueue — declarations`, attached to `bool mergePeerDwells(uint32_t target_id, uint32_t peer_mask,` (line 99)

```text
Merge a peer's clear-LoS dwelled vantage-index mask into the target with
`target_id` (team quota: the union of everyone's dwells counts toward
success). `ring` is the locally generated vantage ring for that target —
canonical positions for newly credited indices are appended to
visited_vantages so isVantageVisited() skips angles a peer already
captured. Idempotent; returns true iff any new index was credited.
No-op on DONE or unknown targets.
```

## include/explo_planner/tree_detector.hpp

### tree-semvoxel

**How SemVoxel fields are built** — in `SemVoxel — declarations`, attached to `struct SemVoxel {` (line 37)

```text
One occupied voxel with the semantics the detector needs. The node builds
these from a ScovoxMap voxel: best_class = argmax over semantic_evidence,
class_conf = best_evidence / (sum_evidence + a_unk), evidence = a_occ+a_free,
p_occ = a_occ / (a_occ + a_free). In geometric mode (LiDAR-only maps) the
semantic records are empty: best_class / class_conf stay 0 and are ignored.
```

### tree-vertical-completeness

**Vertical completeness is self-normalising** — in `TreeDetection — declarations`, attached to `float info_deficit          = 0.0f;  ///< combined "not enough info" score.` (line 72)

```text
< the bins span [base_z, top_z], and those bounds are
< themselves the min/max of the cluster, so the first
< and last bin are always filled and the span is
< self-normalising. It therefore only ever detects an
< INTERIOR gap (>= 1/n_height_bins of the height with
< no voxels at all), which a continuous trunk never
< has -- it read exactly 1.00 on all 17 trunks of the
< map-test-2 bag. It is reported for diagnostics but
< w_vertical defaults to 0: it is not a usable
< occlusion signal in its current form.
```

### tree-detector-mode

**use_semantics selects the front-end** — in `TreeDetectorConfig — declarations`, attached to `bool use_semantics = true;` (line 96)

```text
true  => semantic front-end (veg_class / min_class_conf gates, trunk band
         relative to each cluster's base).
false => geometric front-end for LiDAR-only maps: occupancy-only gate plus
         the "Geometric mode" knobs below; veg_class / min_class_conf /
         trunk_band_* are ignored. Scoring and the info-deficit predicate
         are identical in both modes.
```

### tree-min-trunk-voxels

**How min_trunk_voxels was chosen** — in `TreeDetectorConfig — declarations`, attached to `float    min_height     = 1.5f;   ///< reject clusters shorter than this (m).` (line 113)

```text
< old default of 8 admitted noise: on the
< map-test-2 bag every real trunk carried
< 112-717 trunk voxels while every false
< positive carried 9-33, so this single
< gate separates them cleanly. Scale it
< down for coarser maps / smaller stems
< (it counts voxels, not metres).
```

### tree-geometric-terrain

**Geometric-mode terrain and its limits** — in `TreeDetectorConfig — declarations`, attached to `float terrain_cell_m  = 1.0f;   ///< XY cell of the min-z terrain grid (m).` (line 124)

```text
Terrain: min occupied z per XY cell, median-filtered over the 3x3 cell
neighbourhood, bilinearly interpolated at query positions. Known
limitations: (a) grades beyond ~2*ground_margin_m/terrain_cell_m (~38 deg
at defaults) still leak ground voxels past the margin gate; (b) cells that
only ever saw canopy (never the ground under it) over-estimate terrain and
may cost the tree its lowest voxels; (c) two stems whose surfaces fall
within cluster_tol_m merge into one slice cluster that can fail the
linearity gate, dropping both (semantic mode returns one merged detection
instead); (d) a tree fragment separated from its stem by a map gap wider
than cluster_tol_m is not attached and goes uncounted.
```

### tree-azimuth-bins-coarse

**Why angular coverage uses 8 bins** — in `TreeDetectorConfig — declarations`, attached to `int   n_azimuth_bins = 8;     ///< K sectors for angular coverage.` (line 151)

```text
8 sectors (45 deg). Kept coarse on purpose: a thin trunk only presents
~2*pi*r/voxel distinct surface voxels around its circumference, so too many
bins can never all fill and a fully-circled trunk would keep a high deficit,
breaking the self-closing loop. 8 is fillable at typical trunk r / map res.

Coverage measured from map geometry is only ever a PROXY, and a fragile one
-- it depends entirely on the axis estimate being the true axis rather than
the centroid of whatever happens to have been observed (see fitCrossSection).
The node's use_bearing_coverage measures the same quantity directly, from
the bearings the robot actually viewed the trunk from, and should be
preferred whenever a pose is available.
```

### tree-info-deficit-shared

**One deficit formula for detector and node** — in `infoDeficit`, attached to `float infoDeficit(const TreeDetectorConfig& cfg, float coverage,` (line 173)

```text
Combine the three information terms into the deficit using cfg's weights
(auto-normalised, so the result stays in [0, 1] however they are set).
Exposed because tree_detector_node re-scores a detection against a coverage
value the map alone cannot supply: the set of robot bearings a trunk has
actually been viewed from. Keeping one implementation means the node's
verdict and the detector's stay on the same scale.
```

### tree-emit-gate

**The bearing-coverage emission gate** — in `EmitGate — declarations`, attached to `struct EmitGate {` (line 185)

```text
Per-tree state behind tree_detector_node's decision to publish a TreeTarget.
Lives here rather than in the node so the state machine is unit-testable
without a ROS graph -- the node owns one gate per tracked tree and feeds it
exactly one observation per scan.

The gate answers a different question from the detector's `under_informed`.
The detector scores a SNAPSHOT ("is this trunk under-observed right now?"),
which under bearing coverage is trivially yes for every tree at first sight:
a track's bearing history starts empty, so coverage reads 1/n_azimuth_bins
and the deficit clears any sane threshold. Emitting there means "nominate
every tree the instant it is detected", which makes deficit_thresh
decorative. The gate instead waits until the bearing history has STOPPED
GROWING -- the robot has finished passing this tree -- and only then asks
whether it is still under-covered. A tree the robot happened to walk around
closes itself and is never nominated.
```

### tree-step-emit-gate

**When stepEmitGate fires** — in `stepEmitGate`, attached to `bool stepEmitGate(EmitGate& g, bool under_informed, bool has_bearing,` (line 228)

```text
Advance confirmation and decide whether to publish this tree now. Returns
true exactly once per tree, on the scan where all of these first hold:
  * `under_informed` (on the coverage the caller scored the tree with);
  * confirm_ticks consecutive under-informed scans -- a well-observed read
    resets the streak, so a half-built trunk cannot fire on a fluke;
  * the bearing history has been static for settle_ticks scans, i.e. the
    robot has finished passing the tree. Skipped when `has_bearing` is false
    (no pose, so the verdict came from map geometry and there is no bearing
    history to settle) or settle_ticks <= 0 (deferral disabled).
Sets g.emitted on the true return, so emit-once needs no caller bookkeeping.
```

## include/explo_planner/vantage_planner.hpp

### vantage-deterministic-order

**Vantage generation is deterministic** — in `VantagePlanner — declarations`, attached to `std::vector<CandidateViewpoint> generateVantages(` (line 49)

```text
Generate cfg_.n_vantages viewpoints evenly spaced on the standoff circle
around `center`. Each viewpoint's yaw faces the trunk centre and
is_vantage is set. Z is cfg_.robot_z. Angular order is deterministic
(i = 0..n-1 at start_angle + i * 2π/n), so the same trunk yields the same
vantage positions every tick (the queue's visited-by-proximity tracking
relies on this stability).
```

### vantage-line-of-sight

**The line-of-sight occlusion test** — in `VantagePlanner — declarations`, attached to `bool lineOfSightClear(const Eigen::Vector3f& from,` (line 58)

```text
Line-of-sight test from `from` to the trunk at `center` (radius). Marches
a ray at the sightline height (from.z) toward the trunk axis and returns
false if any known-occupied voxel (p_occ >= occ_stop) is hit *before*
reaching within (radius + one voxel) of the axis — voxels at or inside the
trunk surface are the trunk itself (an expected hit, not an occluder).
Unknown space (Beta(1,1) prior, p_occ = 0.5) never blocks. An empty map is
trivially clear.
```

## launch/exploitation_experiment.launch.py

### launch-terrain-relative-z-flat

**terrain_relative_z on flat sites** — in `generate_launch_description`, attached to `DeclareLaunchArgument(` (line 64)

```text
Overrides shared_params.yaml, which pins terrain mode for the field
(that AO has ~14 m of relief). Set false on a FLAT site — every
hmr_sim world on a ground_plane qualifies. It is not just an
optimisation there: terrain mode places a vantage at mapped ground +
clearance, and the ground on the FAR side of a trunk is exactly what
the trunk occludes, so those vantages are rejected `noground` and the
trunk closes PARTIAL with the ring half-driven — with no approach
march left to fix it, since the robot is already standing at the ring.
Flat mode takes the fixed absolute sightline height and never looks
the ground up, so the failure cannot occur; on a ground_plane world
the two heights are the same number anyway.
```

## launch/multi_robot_exploration.launch.py

### launch-reconnect-alias-empty

**Deprecated rendezvous_enabled alias** — in `launch_setup`, attached to `reconnect_enabled = LaunchConfiguration("reconnect_enabled").perform(context)` (line 44)

```text
Master switch for the reconnect subsystem. `rendezvous_enabled` is the
deprecated alias and defaults to EMPTY, not "true", so that "the caller
asked for it" is distinguishable from "the caller said nothing" — the
node resolves the same way, and passing both unconditionally would make
the alias win every launch and silently shadow the real argument.
```

### launch-alias-stripped

**Carry the stripped alias value** — in `launch_setup`, attached to `reconnect_enabled = rendezvous_enabled.strip()` (line 56)

```text
The STRIPPED value, matching what was tested one line above. The
truthiness test below is an exact membership check, so carrying the
padding through would read `rendezvous_enabled:=" true "` as false and
turn the subsystem OFF for a caller who asked to turn it on — the one
outcome this alias exists to prevent.
```

### launch-own-map-and-minpos

**Own-map topics and MinPos coordination** — in `launch_setup`, attached to `"coordination_enabled":` (line 92)

```text
dscovox_topic / planning_map_topic intentionally left
at their defaults so each planner reads its own
robot's per-robot fused view.
Multi-robot coordination on. The MinPos branch in the
planner's doPlan walks claim_matching against peer
intents on /exploration/intents.
```

### launch-ordered-fleet

**The ordered fleet list** — in `launch_setup`, attached to `"team_robot_names": robots,` (line 110)

```text
THE ORDERED FLEET. Position in this list IS the robot id
that the in_range_mask, the knowledge bitmasks and the
rendezvous proposer rule (robot 0 proposes) all address
by. It is the same `robots` list this loop iterates, so
every node in one launch agrees on it by construction.

Added 2026-09-16. Without it fleet identity is
unconfigured, and the rendezvous scheduler below calls
requireFleetIdentity and refuses to start — so this launch
could not actually run the `hybrid` arm it declares as its
default. Unconfigured remains legal in the node (it is
what a single-robot launch does); it is just not something
a multi-robot launch should ever leave to chance.
```

### launch-reconnect-modes

**The reconnect_mode arms** — in `launch_setup`, attached to `"reconnect_mode": reconnect_mode,` (line 124)

```text
Mesh reconnection manoeuvre on a robot-carried radio
team: rendezvous (drive to the cell and time the whole
team agreed while still connected), pursuit (chase the
missing peer's trail on a budget), or hybrid (chase while
the agreed meeting is not due yet, keep it when it is).
The pursuit_* budgets come from shared_params.yaml.
```

### launch-schedule-interlock

**Why rendezvous_schedule_enable is passed** — in `launch_setup`, attached to `"rendezvous_schedule_enable":` (line 131)

```text
REQUIRED BY rendezvous AND hybrid, and interlocked in the
node: both arms ARE the agreed meeting, so with the
scheduler off there is no agreement to make and the cell
silently runs as a different arm (rendezvous degrades to
the off arm, hybrid to the pursuit arm). The node treats
that combination as fatal rather than let it produce a
healthy-looking run carrying no treatment, so this has to
be passed here — it defaults to false in the node, where
false is correct for the arms that do not schedule.
```

### launch-proximity-stop

**Coordinated proximity stop** — in `launch_setup`, attached to `"proximity_stop_enabled":` (line 143)

```text
Coordinated proximity stop: yield (cancel the nav goal,
hold) when a lex-smaller teammate is moving nearby. In
sim the guard runs off the 1 Hz intent heartbeats; on
hardware add the peers' localiser topics via
proximity_peer_pose_topics in the yaml.
```

### launch-max-steps-default

**The max_steps default of 500** — in `generate_launch_description`, attached to `DeclareLaunchArgument("max_steps", default_value="500",` (line 174)

```text
C4: 500. This is the campaign launch path, and 100 was a fifth of
the budget every reported run actually had.
```

## src/candidate_generator.cpp

### candgen-occupied-filter

**Occupied-candidate filter by mode** — in `CandidateGenerator::generate`, attached to `if (map && (terrain || pos.z() > cfg_.ground_z)) {` (line 49)

```text
Filter occupied candidates (only when 3D map is available).
Flat mode: skip the check for ground-level voxels — they are always
occupied. Terrain mode: the candidate sits z_clearance above the
detected ground, so an occupied voxel there is a real obstacle
(canopy/overhang/wall) — always check.
```

### candgen-frontier-ground-ref

**Frontier ground search reference** — in `CandidateGenerator::addFrontierCandidates`, attached to `const float z = terrain ? terrainZ(fc.x(), fc.y(), fc.z(), *map)` (line 93)

```text
Terrain mode references the ground search to the centroid's OWN z (a
distant frontier can sit many metres above/below the robot); the
centroid z itself is the fallback — frontiers border unobserved
columns, so a missing ground there is expected, and the centroid is a
real free voxel at a plausible height already.
```

### candgen-clamp-ingest-band

**Clamp candidates into the ingest band** — in `CandidateGenerator::terrainZ`, attached to `if (!(cfg_.roi_max_z >= cfg_.roi_min_z)) return z;` (line 115)

```text
Keep the candidate inside the band the map was actually ingested over — see
CandidateConfig::roi_min_z. ground + z_clearance can escape it upward
because the frontier path references the search window to the centroid's
own z, not the robot's. Clamp rather than reject: the clamped point is
still the best viewpoint for that column and its rays now walk ingested
space, whereas dropping it loses a long-range frontier target outright.
Guard the degenerate band (std::clamp is UB when hi < lo).
```

## src/cell_world.cpp

### cellworld-fnv1a-local

**Why a local FNV-1a hash** — in `Fnv1a — declarations`, attached to `struct Fnv1a {` (line 17)

```text
FNV-1a 32-bit, the same construction fleet_identity uses and for the same
reason: this value is compared between processes, so it may not be
std::hash. Kept local rather than shared because the two hash different
things and a shared helper would invite hashing them into one value, which
would stop an operator being told WHICH half of the config disagrees.
```

### cellworld-self-id-required

**Cell world requires a self id** — in `CellWorld::configure`, attached to `if (self_id < 0 || self_id >= kMaxTeamSize) {` (line 195)

```text
self_id is required, not optional. known_by is a bitmask addressed by id;
with no id every bit this robot would set is bit-nothing, and the
knowledge gate downstream would compute a correct-looking answer from an
always-empty mask. Refusing here is what keeps that from being silent —
callers gate on fleet identity being configured before enabling this.
```

### cellworld-frontier-veto

**Frontier fraction vetoes promotion only** — in `CellWorld::classify`, attached to `const double ff = o.frontierFraction();` (line 248)

```text
The frontier count vetoes PROMOTION only. It is the volumetric check the
2.5D column measure cannot do (unknown pockets behind a trunk), but it
is also the noisier of the two — it counts absent 6-neighbours, so the
ROI boundary contributes to it. Using it to demote as well would put the
flap back in through a channel the hysteresis band does not cover.
A cell with observed columns but no observed voxels cannot exist, but
frontierFraction()'s "cannot measure" sentinel is negative and would
sail under any threshold, so the veto is written to require a real
measurement rather than to trust that it got one.
```

### cellworld-relayed-raise-only

**Relayed beliefs are only raised** — in `CellWorld::classify`, attached to `return exploredRank(fresh) >= exploredRank(current) ? fresh : current;` (line 265)

```text
The current belief came from a peer that was there. My own map saying
"unknown" is not evidence against that — it is evidence I have not been.
So a relayed belief may only ever be RAISED by what I see, never lowered.
Equal rank still adopts, because upgrading provenance from relayed to
first-hand is real information for the merge guard table.
```

### cellworld-shared-hash-count

**Cell count in the shared hash** — in `CellWorld::sharedHash`, attached to `f.i64(static_cast<int64_t>(cells_.size()));` (line 336)

```text
The cell COUNT is folded in first, and it is not redundant with the
per-cell bytes: an empty world and a world of one UNSEEN cell would
otherwise differ only by one zero byte, and — more to the point — two
grids of different size are not comparable at all, so their digests must
not be able to collide by accident. configHash() covers the geometry
properly; this is the cheap guard for a caller that compares digests
without having compared grids.
```

### cellworld-local-priority

**mTARE local-priority rule** — in `CellWorld::mergeWire`, attached to `if (n_priority > 0 && local == CellStatus::EXPLORING) {` (line 417)

```text
mTARE's local-priority rule. Refused ENTIRELY — not even the known_by
OR — because while this robot is standing in a cell and actively
exploring it, a peer's claim about that ground is the one claim we have
positive reason to distrust, and crediting the peer with knowledge of a
status we are about to change would suppress exactly the reconnection
that would tell it otherwise.
```

### cellworld-agree-mask-asymmetry

**Known-by merge on agreement** — in `CellWorld::mergeWire`, attached to `if (normaliseForWire(local) == w.status) {` (line 430)

```text
Statuses AGREE on the wire: nothing to adopt, but the peer's knowledge
composes with ours. The peer's own mask is ORed in as well, and that is
what makes the knowledge gate work at N>=3 without hearing from
everybody: if A and B agree a cell is COVERED and B's mask says C has it
too, A can conclude the whole team has it.

Note the ASYMMETRY with adoption below, which is deliberate. Here the
peer's mask is a claim about the SAME status we already hold first-hand
or otherwise, so composing it adds one level of hearsay. When we ADOPT a
status we are already taking the peer's word for the status itself, and
crediting third parties on top of that compounds two levels — so
adoption resets to {self, sender} instead. Over-crediting is the
expensive direction: it makes the knowledge gate suppress a reconnection
that was needed, and nothing later can recover the lost information.
```

### cellworld-exploring-retraction

**Peer EXPLORING over COVERED_BY_OTHERS** — in `CellWorld::mergeWire`, attached to `if (maskHas(c.known_by, sender_id)) adopt = CellStatus::EXPLORING_BY_OTHERS;` (line 464)

```text
THE ONE PLACE THIS DELIBERATELY STRENGTHENS THE PLAN'S TABLE (§3.2).

The plan makes peer-EXPLORING apply to a local COVERED_BY_OTHERS
unconditionally. That is right for the case it was written for — the
SAME peer retracting, because its own hysteresis released the cell —
and full-state broadcast means the peer's latest census is what
arrives, so retractions must be trackable or our view of that peer
can never come back down.

But applied unconditionally it also lets a THIRD robot's staler
EXPLORING undo a COVERED we adopted from someone else, which is a
regression of the census under mere reordering — the thing rule 1
and the more-explored ordering exist to prevent. known_by already
records who we took the COVERED from, so the two cases are
distinguishable and there is no reason to conflate them: honour a
retraction from a robot that is in the mask, refuse a contradiction
from one that is not. At N=2 there is only ever one peer, so this is
identical to the plan's rule; it only bites at N>=3, where the plan's
version is wrong.
```

### cellworld-intercept-self

**No intercept candidates for self** — in `CellWorld::interceptCandidates`, attached to `if (robot_id == self_id_) return out;` (line 506)

```text
Asking where to look for OURSELVES has no answer, and the honest one is
"nowhere". It is not "nowhere" by accident: self is in the known_by of
every cell we have observed or adopted, so without this the query would
return most of the map, and a caller that looped over the fleet without
excluding itself would get a large, plausible, entirely spurious search
list for the one robot that is definitely not missing.
```

### cellworld-column-binning

**One helper bins columns for both counts** — in `censusFromMap`, attached to `auto axis_index = [&](int64_t c, float lo, int ncells) -> int {` (line 641)

```text
A voxel column has integer coord (cx, cy) and covers
[cx*res, (cx+1)*res) — Bonxai's coordToPos returns the LOW corner. Bin it
by that corner, the same floor() convention as ingest.

ONE helper does the binning for both the denominator (total_columns) and
the numerator (observed_columns), rather than the counts being computed
analytically and the observations through CellGrid::idAt. Those two would
drift: at min_x = -15 and res = 0.2 the boundary column is
-75 * 0.2 == -15.000000000000002 in double, which floors to cell index -1,
but narrowed to float it is exactly -15.0f and floors to 0. One path would
then count a column the other did not, and every cell on the ROI's west
and south edges would report an unknown fraction slightly below the truth
— small enough to look like a threshold that needed tuning. Sharing the
arithmetic makes the two agree by construction.
```

### cellworld-packed-column-keys

**Packed column keys** — in `censusFromMap`, attached to `std::unordered_set<uint64_t> observed;` (line 696)

```text
Packed (x, y) column keys, deduplicated: x zero-extended into the high 32
bits, y into the low. Bijective for 32-bit coords, and the shift runs on
uint64_t because left-shifting a negative signed value (any column west of
the origin) is undefined behaviour. Same construction as
unknownColumnFraction, deliberately, so the two measures agree.
```

## src/coordination.cpp

### coord-bound-claim-radius

**Bounding the peer-advertised claim radius** — in `Coordination::onIntent`, attached to `claim.radius_m = std::isfinite(msg.claim_radius_m) && msg.claim_radius_m > 0.0f` (line 42)

```text
Bound the peer-advertised radius. claimMatching() deliberately tests each
claim at the radius its CLAIMER advertised (the disc sizes are
phase-dependent — ~10 m exploring, ~0.75 m holding one vantage angle), which
means an unbounded value straight off the wire lets one peer veto an
arbitrarily large region of our candidate set: +inf makes r2 infinite and
EVERY candidate match, so the robot yields every goal to that peer and stops
exploring entirely. Non-finite or non-positive becomes 0, which
claimMatching already reads as "peer sent nothing usable, use our own notion
of the same goal"; anything larger than our own exploration disc is clamped
to it. A same-version peer is never affected — its radius is either
coord_claim_radius_m or the smaller vantage disc.
```

### coord-claim-expiry-local-clock

**Claim expiry on the local clock** — in `Coordination::onIntent`, attached to `const float ttl = std::isfinite(msg.ttl_sec)` (line 64)

```text
Expiry = LOCAL receipt time + ttl, so prune(now) compares two timestamps
from the SAME clock. Building expiry from the producer's header.stamp
breaks on real robots: fleet clocks are not synchronised (offsets of
hours have been observed in the field), so a peer clock ahead of ours
made its claims immortal and a peer behind by > ttl made them expire on
arrival — both silently. The TTL semantic is "how long since we last
HEARD this peer", which only needs the local clock.

ttl_sec is peer-advertised too, and unbounded it is the same failure one
field over: a claim prune() can never expire is a permanent veto. Clamp into
[0, kMaxClaimTtlSec]. 0 makes expiry == now_local, so prune() drops the
claim on the very next tick — the safe direction for a peer whose TTL we
cannot read. Both ends matter: from_seconds() overflows its int64 nanosecond
count on a wild magnitude, and non-finite must not reach it at all.
```

### coord-claimer-radius

**Match at the claimer's radius** — in `Coordination::claimMatching`, attached to `const float r = (c.radius_m > 0.0f) ? c.radius_m : match_radius_m;` (line 134)

```text
Test against the radius the CLAIMER advertised, not our own. The claim
radius is the size of the region that peer is occupying, and it is
phase-dependent: an exploration claim is ~8-10 m ("I'm driving to this
area"), an exploit vantage claim is ~0.75 m ("I'm holding this one angle
around a trunk"). Evaluating every claim at the receiver's own scale
meant an exploring robot vetoed an 8 m disc around a peer that was
merely dwelling at a tree, and an exploiting robot under-vetoed a peer's
exploration goal. radius_m <= 0 means the peer sent nothing usable
(older node), so fall back to our own notion of "same goal".
```

### coord-unstaged-expiry-here

**Expiry tested in the dwell barrier** — in `Coordination::firstUnstagedExploitPeer`, attached to `if (c.expiry <= now) continue;` (line 194)

```text
Expiry is tested HERE instead of being left to prune(), unlike every
other lookup in this class. The caller is parked in EXPLOIT_DWELL, a
state whose whole point is that it does not re-plan, so the PLAN tick —
and with it prune() — never fires while the barrier is up. A peer that
died mid-drive would leave an unstaged claim sitting in the table and the
robot that DID arrive would dwell on it forever. With this check the
receipt-time TTL is the release path: a silent peer holds us for one TTL
(~5 s, one heartbeat plus margin) and no longer.
```

### coord-staged-producer-flag

**Staged is the producer's word** — in `Coordination::firstUnstagedExploitPeer`, attached to `if (!c.staged) return &c;` (line 205)

```text
"Staged" is the producer's own word: it sets the flag when it enters
EXPLOIT_DWELL, after nav arrival AND the post-arrival rotation settle, and
clears it on every claim it publishes while driving. Each ~1 Hz heartbeat
carries the current value, so an inbound peer's claim flips staged by
itself and the barrier progresses without a re-plan on our side.

Nothing geometric is consulted here any more. Comparing the claim's
robot_pos against its own goal_pos within a tolerance answered "is it
near that point", which is not the question: a peer that had arrived in
XY but was still rotating to its capture yaw passed the test ~5 s before
it was actually settled, and peers that released on it lost most of the
simultaneous window (~3 s of an 8 s dwell overlapped); and an APPROACH
hop — an exploit claim whose goal IS an intermediate waypoint — passed it
while the peer stood nowhere near a vantage. Which of a peer's exploit
hops is a vantage it is HOLDING is knowable only at the producer.
```

### coord-staged-probe-retention

**Staged probe uses the graced bound** — in `Coordination::stagedExploitPeerWinning`, attached to `if (!withinRetention(c, now)) continue;` (line 230)

```text
Retention filtered here as in firstUnstagedExploitPeer(), but on the
GRACED bound, not raw expiry — this probe is a veto, and a parked peer
whose heartbeats sat undelivered is still parked (see the header). The
check is redundant when called right after a plan-tick prune(); it is
kept so the probe is correct from any call site, and so a future caller
in a non-planning state cannot be vetoed by a claim past even the grace
window.
```

### coord-staged-only-contest

**Only staged peers contest by position** — in `Coordination::stagedExploitPeerWinning`, attached to `if (!c.staged) continue;` (line 240)

```text
Staged claims ONLY. A parked peer is about to re-plan from a standstill
beside this ring, so if it is closer to this angle it wins the race for it
and we may as well concede now, before either of us moves — that is the
collision this probe prevents.

A DRIVING peer must never block a candidate this way. It contests through
its claim disc (claimMatching()) and nothing more: a robot approaching the
ring from far away is farther from EVERY vantage on it than a peer already
circling the trunk, so a position contest against a mover would deny it
the whole ring and serialise an exploitation that is meant to run in
parallel. The mover's `robot_pos` is also up to a heartbeat stale, so the
comparison would be against a pose it has already left.
```

## src/cost_grid.cpp

### costgrid-unknown-traversable

**Unknown cells stay traversable** — in `CostGrid::build`, attached to `const auto& data = planning_map.data;` (line 47)

```text
Sample the obstacle layer once. Only *known* occupied/inflated cells
(value >= obstacle_threshold) are impassable. Unknown cells (value -1)
are treated as traversable so the flood can path through unexplored
space — the planner's per-cell isCellFree() filter already rejects
candidates that sit on unknown cells, so reachability's job is only
to catch free pockets sealed off by *known* obstacles, not to block
paths through the exploration frontier.
```

### costgrid-conditional-seed

**Why the flood seed is conditional** — in `CostGrid::floodFrom`, attached to `bool can_seed = !blocked_[idx(sx, sy)];` (line 87)

```text
Seeding the source at cost 0 is conditional, and it used to be
unconditional, which made this class lie on exactly the input it is
supposed to be defensive about. build() marks EVERY cell blocked when the
planning_map is malformed (see the data.size() != n guard there), so the
flood is empty by construction — but the seed was still written, and it was
then the ONLY finite cost in the grid: reachable(robot_pose) answered true
and reachedCellCount() answered 1 on a map where nothing whatsoever is
reachable. A reachability structure may answer "no"; it must never answer
"yes" off a map it has already rejected. (2026-09-18)

The test is NOT simply "is the source cell traversable", because a blocked
source is a legitimate and routine state: the map is inflated by the body
radius, so a robot in a dense stand genuinely stands on an inflated cell,
and the flood must still start from there — the relaxation below already
refuses to pass through any *other* blocked cell, so starting on inflation
does not let a path run through it. The honest question is whether the
flood has anywhere at all to go: seed when the source is itself traversable
OR when at least one of its eight neighbours is. A malformed map fails both
and leaves the whole grid at kInfCost; the robot-in-inflation case passes
the second and is unchanged.
```

### costgrid-radius-cap-unbounded

**Radius cap and no diagonal clamp** — in `CostGrid::floodFrom`, attached to `float cap = radius_cap_m;` (line 116)

```text
Effective radius cap. Negative / zero / NaN means "no bound" — genuinely
unbounded, i.e. +inf. Do NOT clamp to the grid diagonal: a Dijkstra path
length is a *walked* distance and routinely exceeds the straight-line
diagonal (serpentine corridors, U-shaped rooms), so a diagonal clamp made
reachable cells report kInfCost and the exploitation planner rejected
genuinely drivable vantages as unreachable. Termination does not depend on
the cap — the grid is finite and each cell is relaxed to a strictly
decreasing cost.
```

## src/exchange_drain.cpp

### drain-fleet-sized-vectors

**Unmeasured counters are not drained** — in `stepDrainRelease`, attached to `const size_t n = static_cast<size_t>(fleet_size);` (line 23)

```text
THREE VECTORS, ALL FLEET-SIZED OR THE READING IS NOT A READING. The
counters are empty until dscovox publishes them, and the hold-start
baseline is a copy of whatever they were then — so a meeting that began
before the counters came up has no interval to difference. That is
UNMEASURED, and the safe reading of unmeasured is "not drained": hold to
the cap and say the exchange did not finish, rather than release on the
absence of evidence.
```

### drain-believed-presence

**Presence is the robot's belief** — in `stepDrainRelease`, attached to `if (!team.configured() || id >= team.size()) continue;` (line 40)

```text
BELIEVED PRESENT, which is what the robot actually has. The detector's
false-positive rate (2.948% of scored grid points read direct while the
oracle says down) is inherited here and not claimed away: a robot can
hold for a peer that is not really there. The cap below is what bounds
that, and it bounds it to a wasted wait rather than a stall.
```

### drain-direct-or-relayed

**Direct or relayed peers count** — in `stepDrainRelease`, attached to `const auto& p = team.peer(id);` (line 46)

```text
DIRECT OR RELAYED. The question at this loop is "is there somebody here
to trade maps with", which is a radio statement, and a two-hop peer
answers it: the emulator forwards serialized bytes without deserializing,
so dscovox credits the robot that SENSED the voxels rather than the one
that bridged them (see ScovoxFusionCounters.msg). Gen-32's census
measured relayed rows applying a merge 15.5% of the time against 0.9%
overall — the most productive channel on the team — so a direct-only
filter would let the busiest stream on the meeting keep arriving while
this test declared the exchange finished.

The two flags are disjoint, so this is the whole filter and not half of
one: team_model.cpp's closure loop opens with `if (p.direct) continue;`,
which is why via_relay can never be set on a direct peer.

FINISHED IS NOT PRESENT (test-plan 7). `finished` is sticky and says
nothing about the radio: a peer that finished and drove off is still
finished. Counting it would hold a drained exchange open against a robot
that is not here to deliver anything. (A finished peer still on its way
here does not reach this loop absent: the barrier holds for it before
the hold opens — meeting_attendance.hpp.)
```

### drain-clause-spoke

**Clause 1: did the peer speak** — in `stepDrainRelease`, attached to `if (nowv <= hold_base[k]) {` (line 71)

```text
CLAUSE 1 — DID IT SPEAK AT ALL THIS VISIT. A level against the
hold-start baseline, not a rate. This is the clause that cannot be
optimised away: a peer whose bytes are not crossing the radio and a peer
that has sent everything it has BOTH present a rate of zero over the
window, and dropping this would make the release fire fastest in exactly
the blackout the hold exists to sit through.
```

### drain-clause-rate

**Clause 2: a rate, not zero** — in `stepDrainRelease`, attached to `const uint64_t wb = window.base[k];` (line 82)

```text
CLAUSE 2 — HAS IT STOPPED. A rate, not a zero test: a third of the long
gen-32 N=2 meetings were still gaining when the robot departed, so "the
counter has stopped moving" over-holds, while "the counter has fallen
below R" does not.
```

### drain-nobody-read-all-leaving

**Nobody read, and all peers leaving** — in `stepDrainRelease`, attached to `int here = 0;` (line 91)

```text
NOBODY READ IS NOT EVERYBODY DRAINED. Every `continue` above is a peer
this robot could not read — model unconfigured, id past its end, or
believed not here — and with all of them taken the loop falls out leaving
`drained` at the true it started on, releasing the hold having examined no
one. That is the same absence of evidence `measurable` refuses a few lines
up, arriving through a different door: the hold opens on
max(active, reachablePeerCount), which counts peers this loop is entitled
to skip.

EXCEPT WHEN EVERY PEER HAS SAID IT IS LEAVING (2026-09-23). That is not an
absence of evidence but positive evidence, first-hand or relayed: each one
announced homing or done, and there is nobody left to come and trade maps
with. Holding to the cap for them is the wait the partner protocol exists
to end — a robot stops waiting when its partner says it is leaving. All of
them, not any: one peer that has not said so could still be walking in,
and that is the case this backstop is for. And NOBODY HERE, counted on its
own rather than read off `examined`: the loop above does not run at all
when the counters are unmeasurable, so `examined` is 0 there with a peer
standing on the cell — and a peer that is here and leaving may still have
bytes crossing, which is the counter's question, not this one's.
```

## src/failed_goal_blacklist.cpp

### blacklist-prune-full-scan

**Why prune does a full scan** — in `FailedGoalBlacklist::prune`, attached to `const auto is_stale = [now_sec, ttl_sec](const FailedGoalSite& s) {` (line 48)

```text
Full scan rather than a pop-front-until-fresh loop. The early-break form
assumed insertion order implies age order, which holds only for a
monotonic clock — under sim time a bag restart or a /clock step backwards
stamps a fresh entry with an *older* timestamp than the one behind it, and
the break then left every expired entry after it blacklisting goals
forever. Entries stamped in the future (age < 0) are treated as fresh.
```

### blacklist-expire-not-erase

**Expire, not erase, with retirement** — in `FailedGoalBlacklist::prune`, attached to `for (auto& s : sites_) {` (line 67)

```text
Retirement ON: expire, do not erase. prune() runs every PLAN tick, so a
robot that returns to the same trap after longer than the TTL used to find
the record gone and start counting from 1 again — which made retirement
unreachable in exactly the case it exists for (a trap revisited every few
minutes). Keeping the record costs one struct per distinct failure SITE,
and a site costs a whole failed navigation to create, so the list stays in
the tens over a full run.
```

### blacklist-any-retired-near

**Any retired site, not nearest** — in `FailedGoalBlacklist::isRetiredNear`, attached to `const float r2 = static_cast<float>(radius_m * radius_m);` (line 102)

```text
ANY retired site within the radius, not the nearest one. Nearest-wins was
wrong on two counts. Semantically, retirement is a veto — "held for the
rest of the run" — and a veto is not overturned by a fresher record
happening to sit a few centimetres closer. Mechanically, it did not even
describe the same site the caller had just written: add() clusters into
the FIRST site within radius, not the nearest, so failGoal could increment
site A and then log the retired flag of site B. That flag is what
nav_goal_failed carries into the analysis, so a mismatch there is a
silently wrong event field, not just a confusing WARN.
```

### blacklist-last-fail-excludes-expired

**lastFailTimeNear skips expired sites** — in `FailedGoalBlacklist::lastFailTimeNear`, attached to `const float r2 = static_cast<float>(radius_m * radius_m);` (line 120)

```text
Expired records excluded, to match isNear. Both callers — the amnesty
ordering and the amnesty log line — only ever see candidates that were
SUPPRESSED this tick, so an expired site can never be the subject; letting
one through would only change the answer for a candidate that is not
suppressed at all.
```

### blacklist-amnesty-order

**Amnesty order: retired last** — in `amnestyOrderBefore`, attached to `const bool ra = bl.isRetiredNear(a, radius_m);` (line 137)

```text
Retired last, then least-recently-failed.

The retired partition is not cosmetic. Ordering on fail time ALONE hands
the amnesty valve straight back to the permanent traps that retirement
exists to hold, and does so systematically rather than occasionally: a
retired site is never pruned, so once the robot stops re-failing it its
last_fail_time only gets older, while every ordinary site near it either
gets re-failed (fresh time) or ages out of the blacklist entirely and
stops being a suppressed candidate at all. Give it a few minutes and the
oldest failure in the list is essentially always the confirmed trap. The
one valve meant to rescue a starved planner would then aim it at the
known-unreachable goal first, every single time.

A retired candidate is still reachable as a last resort — that is the
point of amnesty, and it is what keeps the worst case no worse than the
old behaviour — but only once nothing merely-suppressed is left to try.
```

## src/fov_evaluator.cpp

### fov-clip-ray-to-roi

**Clipping a ray to the ROI** — in `clipRayToRoi`, attached to `bool clipRayToRoi(const Eigen::Vector3f& origin,` (line 11)

```text
Clip the observable segment of one ray — the span from the sensor's minimum
range out to max_range — against the XYZ ROI box. Standard slab method:
intersect the per-axis entry/exit intervals with [min_range, max_range] and
keep what survives.

BOTH ends need clipping, and the entry end for two distinct reasons:

 1. A candidate within min_range of an ROI face, firing outward, has its ROI
    exit BEFORE min_range. Clamping only the far end left `ray_start` at
    origin + dir*min_range — outside the box and PAST the clamped far end —
    so RayIterator walked backwards through cells map_cache_ never ingests,
    scoring each as the Beta(1,1) max-uncertainty prior and inflating info
    gain exactly at the ROI boundary. Caught by the (t_exit > t_enter) test.

 2. An origin already OUTSIDE the box on some axis, firing back toward it.
    In terrain mode this is reachable in the shipped config: a frontier
    candidate is snapped to ground + z_clearance searched around the
    CENTROID's z, so it can land up to (ground_search_above + z_clearance)
    above the ingested band. Every near-horizontal ray from there has
    d.z ~ 0, so an exit-only clip skipped the z axis entirely (see 3 below)
    and walked the full max_range through un-ingested space at the prior —
    the same boundary bias, one axis over. CandidateGenerator now clamps
    candidate z into the band as well, so this is belt-and-braces.

 3. d[i] == 0 means the ray is parallel to that pair of faces and never
    crosses either: the axis contributes no bound, and the ray is entirely
    in or entirely out according to the origin alone. Skipping the axis (the
    old behaviour) silently treated "entirely out" as "unconstrained".

Returns false when nothing observable survives, in which case the ray must be
skipped rather than walked.
```

### fov-ssmi-no-hit-term

**The SSMI no-hit term** — in `FovEvaluator::evaluateSSMI`, attached to `if (!occluded) result.total_score += reach * free_kl_acc;` (line 241)

```text
"No hit" event: the ray passes through every voxel on its span and each
cell receives a free observation. Only valid when the walk actually ran
to c_end. On the occlusion-stop path the iteration was TRUNCATED, so the
residual `reach` is not "passed through cleanly" — it is the unmodelled
mass beyond the occluder, and the cells that would carry it were never
visited. Adding the term there credited a ray that demonstrably hit an
occupied voxel with the full free-observation KL of everything in front
of it, biasing the score toward staring at occluders.
```

## src/map_cache.cpp

### mapcache-resolution-check

**Rejecting a degenerate voxel size** — in `checkedResolution`, attached to `double checkedResolution(double resolution, const char* where) {` (line 17)

```text
A non-positive (or non-finite) voxel size makes Bonxai's inv_resolution inf
or NaN, and every posToCoord then does a float->int32 cast on a non-finite
value — undefined behaviour that shows up as garbage coordinates rather than
as a crash. CostGrid::build() already refuses a degenerate grid; do the same
here instead of building an unusable cache. Config error, so fail at
construction with a message naming the value.
```

### mapcache-wire-resolution

**Validating resolution from the wire** — in `MapCache::updateFromScovoxMap`, attached to `const double res = msg.resolution > 0.0f ? static_cast<double>(msg.resolution)` (line 49)

```text
msg.resolution comes off the wire, so validate it here rather than trusting
it into the Grid constructor. `> 0.0f` is already false for NaN and for a
non-positive value (both fall back to the last known-good resolution), but
it is TRUE for +inf — which is the case that reaches Bonxai and makes every
subsequent posToCoord undefined behaviour. See the header for why this
rejects the message instead of throwing like the constructor does.
```

### mapcache-nonfinite-beta

**Dropping non-finite Beta parameters** — in `MapCache::updateFromScovoxMap`, attached to `if (!std::isfinite(vx.a_occ) || !std::isfinite(vx.a_free) ||` (line 75)

```text
Drop non-finite Beta parameters too. p_occ is derived from them, and a
NaN a_occ/a_free silently produced a NaN p_occ that propagated into the
per-voxel scorers and the aggregate metrics (mean_eig / mean_entropy)
for the rest of the run — the ratio test below cannot catch it, since
every comparison against NaN is false and lands on the 0.5f branch only
for the sum, not for the division.
```

### mapcache-column-pack

**Packing columns into one key** — in `MapCache::unknownColumnFraction`, attached to `std::unordered_set<uint64_t> observed;` (line 312)

```text
One walk over active cells, projecting each to its (x, y) column. The
shift-or pack (x zero-extended into the high 32 bits, y into the low 32)
is bijective for 32-bit coords, so distinct columns never collide. The
shift runs on uint64_t: left-shifting a negative signed value (any column
west of the origin, c.x < 0) is undefined behaviour in C++17.
```

## src/metrics_logger.cpp

### metrics-csv-open-fails-loudly

**Failing loudly on an unwritable CSV** — in `MetricsLogger::MetricsLogger`, attached to `if (!file_.is_open()) {` (line 12)

```text
Fail loudly. std::ofstream does not throw by default, so an unwritable
output_csv (missing parent directory, read-only mount, bad permissions)
used to leave every subsequent `file_ << ...` a silent no-op: the run
completed, the node logged "Step N logged" for every step, and the
experiment produced no data at all. There is nothing to salvage from a
metrics run whose metrics cannot be written, so refuse to start.

The open test alone was NOT enough, which is why noteStreamState() exists
below: a successful open says nothing about the writes that follow it, and
the two cases that matter both open cleanly. "/dev/full" accepts the open
and rejects every write; a disk that fills at step 2000 of 3000 accepts the
first 2000. Both reproduce the exact silence this constructor was written
to end, one flush later. (2026-09-18)
```

### metrics-stream-state-sticky

**Sticky first-wins stream error** — in `MetricsLogger::noteStreamState`, attached to `if (!error_.empty()) return;` (line 33)

```text
Sticky and first-wins. Once failbit or badbit is set every subsequent `<<`
is a no-op and the stream stays failed for the rest of the run, so checking
per row would otherwise restate the same failure on every one of the
thousands of ticks that follow it. fail() is the right predicate rather
than bad(): a formatting failure loses the row just as completely as an I/O
error does, and both leave a CSV that no longer matches its own header.
```

### metrics-sanitize-field

**Field sanitising and the empty field** — in `sanitizeField`, attached to `std::string sanitizeField(const std::string& s) {` (line 51)

```text
Replace anything that would change the column count. An empty string stays
EMPTY -- there is no "-" sentinel, and the comment here used to promise one
the code has never written. That promise is worse than no comment: a parser
written against it tests `field == "-"` for "not pursuing", never matches,
and classifies every non-pursuing row as a chase with an unnamed quarry.
The dead `if (s.empty()) return "";` that sat here -- a branch returning
exactly what the loop below would have returned -- was the tell.
```

### metrics-append-only-schema

**Why CSV columns are only appended** — in `MetricsLogger::writeHeader`, attached to `<< "state,reconnect_range_to_goal_m,reconnect_elapsed_sec,"` (line 76)

```text
Appended, never inserted. Every reader in this tree resolves columns
by header name — csv.DictReader in the python, and a header-scanning
awk in run_explo_sim_rviz.sh — so inserting would not in fact break
any of them; a comment here used to claim otherwise and was wrong.
The rule is kept anyway, for the readers that are NOT in this tree.
A campaign archive outlives the binary that wrote it and gets opened
by whatever is to hand months later, and a positional read of an
old file against a new schema does not fail, it silently returns the
wrong column. Appending costs one out-of-place block of names; that
is the whole price, and it is paid below three times over.
```

### metrics-plan-rej-visited-placement

**Where plan_rej_visited sits** — in `MetricsLogger::writeHeader`, attached to `<< "plan_rej_visited\n";` (line 103)

```text
v8. Belongs immediately after plan_rej_blacklist by meaning — it is
that column's visited half — and is here instead, for the same
reason as every block above: inserting it where it reads would shift
FIVE columns under every positional reader of every banked run —
plan_rej_minpos, plan_stall_ticks, pursue_peer, pursue_quarry_live,
team_complete, which is everything between the insertion point and
the end of the header. (This said "eight" until 2026-09-18; count it
from the header above, which is the only place the order is real.)
Five is not a smaller argument than eight. One shifted column is
enough to make every banked run's positional reader wrong.
```

### metrics-pursue-peer-sanitised

**Sanitising the robot id field** — in `MetricsLogger::logStep`, attached to `<< sanitizeField(m.pursue_peer) << ","` (line 172)

```text
The only free-text field in the row that comes from configuration
rather than from this file, so it is the only one that can carry a
separator. A robot id with a comma in it would shift every column to
its right by one on that row and nowhere else — the worst kind of
corruption, because the file still parses. Substituted, not quoted:
a quoted field would be correct CSV but would break the awk
header-scanner in run_explo_sim_rviz.sh, which does not implement
quoting.
```

### metrics-check-after-flush

**Checking stream state after the flush** — in `MetricsLogger::logStep`, attached to `noteStreamState("row");` (line 185)

```text
Checked AFTER the flush, not after the insertions: the row can sit whole in
the stream buffer with every bit clear and only fail on the way to the
device, which is precisely how an ENOSPC presents. Flushing per row is
already this class's contract (a killed run must keep the steps it wrote),
so this adds a branch, not a syscall.
```

## src/plan_map_query.cpp

### planmap-data-size-check

**Buffer size versus grid metadata** — in `planMapCellAt`, attached to `if (m.data.size() !=` (line 16)

```text
info.width/info.height bound the INDEX computed below, but they are only a
claim about the buffer -- nothing in nav_msgs makes data.size() agree with
them. A publisher that fills in the metadata and then sends a short (or
empty) data vector produces a grid that passes every bounds test here and
still reads off the end: a 1140x1140 map carrying 1140 bytes segfaulted the
planner under AddressSanitizer. Treat the disagreement as "this grid has no
data" rather than trusting the header, which is exactly the test
CostGrid::build() already applies to the same message (cost_grid.cpp), so
the two agree on what a malformed planning_map is. (2026-09-18)
```

### planmap-roi-data-size-check

**Malformed grid in the ROI query** — in `unknownFractionInRoi`, attached to `if (m.data.size() !=` (line 55)

```text
Same buffer-vs-metadata mismatch planMapCellAt() screens above, and it
matters more here: the loop below indexes every cell of the clipped ROI, so
a short data vector walks off the end for the whole rectangle rather than at
one cell. -1.0 is already this function's "cannot be measured" answer (the
node's coverage-termination check treats it as "no reading this tick"), so
a malformed grid degrades to no reading instead of to a crash -- the
conservative direction, since a fabricated unknown-fraction would feed the
DONE criterion. (2026-09-18)
```

## src/planner_util.cpp

### navbudget-nan-to-ceiling

**A NaN nav budget takes the ceiling** — in `navBudgetSec`, attached to `if (std::isnan(raw)) raw = max_sec;` (line 33)

```text
A NaN distance (or speed, or safety factor -- std::max does not sanitise
NaN either) used to survive the clamp: std::clamp is written as
`v < lo ? lo : hi < v ? hi : v`, both comparisons answer false against NaN,
and the NaN is returned unchanged. That is the worst possible failure for a
WATCHDOG, because every later `elapsed > budget` test is then false too --
the NAVIGATE timeout never fires, the robot sits on a dead goal, and the
cell runs to max_steps. In the campaign record that is indistinguishable
from a genuinely slow cell, so a censored run is scored as a completed one.
An infinite raw budget already landed on the ceiling via the clamp; this
makes NaN do the same, which is the conservative reading of "we do not know
how far it is": still bounded, still fires, and never cuts a legitimate
long drive short at the floor. (2026-09-18)
```

### navbudget-ceiling-wins

**The ceiling wins over the floor** — in `navBudgetSec`, attached to `return std::clamp(raw, std::min(min_sec, max_sec), max_sec);` (line 46)

```text
The ceiling wins over the floor, for exactly the reason pursuitBudgetSec
spells out below: nothing orders min_sec against max_sec (they are separate
parameters), std::clamp with lo > hi is undefined behaviour, and in practice
the floor won -- a transposed pair produced a budget LONGER than the ceiling
the caller asked for, which is backwards for a timeout. Kept written the
same way in both functions so the two cannot drift apart.
```

### next-occurrence-ceiling-division

**Ceiling division for the next occurrence** — in `nextAgreedOccurrence`, attached to `const long long k =` (line 81)

```text
Ceiling division. (not_before_ms - t_meet_ms) is strictly positive by the
test above and interval_ms is strictly positive by the test above that, so
both operands are non-negative and the usual signed-truncation trap does
not apply: the +interval-1 form is exact, and it lands on t_meet_ms exactly
when the floor falls on an occurrence.
```

### arrival-shortfall-exact-zero

**The exact zero in the shortfall** — in `arrivalShortfallSec`, attached to `return std::max(0.0, lead_ms / 1000.0 - budget_sec);` (line 103)

```text
std::max, not a branch, and the zero is exact: a robot inside its budget
adds NOTHING to the floor, so every robot that can attend the nearest
occurrence indexes the lattice from the same place and the team cannot
fork. The whole safety argument for putting a per-robot term here at all
rests on this line.
```

## src/proximity_guard.cpp

### proxguard-parked-peers

**Parked peers and the keep distance** — in `ProximityGuard::evaluate`, attached to `const bool parked =` (line 104)

```text
Parked peers are the navigator's obstacle grid's job (not a nav2
costmap — there is none here), and holding against one deadlocks
— but only beyond parked_keep_dist_m. Inside the floor "it parked" is
no licence to drive even closer; the caller's max-hold escape hatch is
the deadlock breaker there.
```

## src/pursuit_predictor.cpp

### pursuit-cell-dist-reuses-costmm

**Reusing the allocator's cost model** — in `cellDistM`, attached to `double cellDistM(const CellWorld& w, int a, int b) {` (line 15)

```text
Metres between two cells, through the allocator's own cost model.

Reused rather than reimplemented for the same reason the rendezvous
scheduler reuses routeCostMm: a second distance function is free to drift
from the first, and the symptom of that drift is a chase aimed at a cell the
allocator would never have put on the tour — invisible in every log, because
both numbers look reasonable in isolation. costMm also carries the
unreachable-cell fallback, so an over-aggressive edge probe degrades the
ranking instead of deleting nodes from the chain.
```

### pursuit-pgo-transition

**The per-step advance probability** — in `pGo`, attached to `double pGo(const CellWorld& w, const std::vector<int>& tour, size_t i,` (line 28)

```text
P(the G transition fires this step) for tour node `i`.

A leg takes `dwell + drive` seconds, and a step is `step_sec` of it, so the
per-step advance probability is the ratio. That makes the sojourn time
geometric with the right MEAN — it is not a claim that the peer's dwell is
memoryless, it is the coarsest model with the correct first moment, which is
all a 5-second-resolution intercept can use.

Returns 0 at the tail: a peer at the end of its tour has nowhere on the
record to advance to. Its mass then sits on the last cell and drains only
through O, so a long-finished tour predicts nothing rather than predicting,
with confidence, that the peer is parked on its last cell forever.
```

### pursuit-hazard-order

**Off-route hazard order and switch-off** — in `PursuitPredictor::step`, attached to `const double survive = cfg.offroute_half_life_sec > 0.0` (line 78)

```text
O first, as a hazard on ALL surviving mass, then G/I split the remainder.
Order matters and this one is deliberate: applying the hazard only to the
mass that stayed put would make a fast tour immune to abandonment, which
is exactly backwards — the peer most likely to have re-planned is the one
that has had time to finish what it was doing.
A non-positive half-life is the hazard switched OFF, not one clamped to an
epsilon — see the Config comment. Written as an exact 1.0 rather than an
exp() of something tiny, because the chain's test case is an exact
binomial and a survive of 1 - 3e-9 is not 1.
```

### pursuit-candidate-horizons

**Scoring candidates at arrival time** — in `PursuitPredictor::predict`, attached to `struct Cand { size_t i; int cell; long long travel_ms; long long horizon_ms;` (line 145)

```text
Scored at (record age + MY drive to it), not at the record age alone:
§3.7's objective is to intercept where the peer will be WHEN I GET THERE,
and the two differ by minutes at these speeds. Nodes BEFORE the anchor are
still candidates — the peer may have been driving toward the anchor rather
than away from it, and dropping them would bias every intercept forward.
```

### pursuit-single-forward-pass

**One forward pass for all candidates** — in `PursuitPredictor::predict`, attached to `std::vector<double> p(peer.tour.size(), 0.0);` (line 180)

```text
A fresh chain per candidate would be O(cands x steps x tour) for exactly
the same numbers; the distribution does not depend on which candidate is
being scored, only on how far it is propagated. So propagate once and read
each candidate off as the pass goes by it.
```

### pursuit-argmax-tiebreak

**The intercept argmax tie-break** — in `PursuitPredictor::predict`, attached to `size_t best = 0;` (line 200)

```text
Argmax, with a total tie-break: probability, then the SOONER arrival, then
the earlier tour position, then the lower cell id. Sooner-first is not
cosmetic — two cells the peer is equally likely to be in are not equally
good chases, because the near one re-forms the link earlier and leaves
budget for a second attempt.
```

## src/reconnect_gate.cpp

### gate-empty-missing-fails-open

**No missing peer fails open** — in `evaluateReconnectGate`, attached to `if (missing.empty()) {` (line 56)

```text
NOBODY TO PRICE IS AN INABILITY TO EVALUATE, NOT A DECISION TO STAY.

The caller reached this gate because the mid-run trigger fired, and the
trigger's notion of "missing" is not this one. The trigger reads the
coordination beacon (livePeerCount / team_last_complete_time_, 1 Hz,
claim-TTL'd); `missing` is built from TeamModel::inComms, a different
topic with its own TTL, a two-way handshake and transitive closure. They
are designed to disagree — 16-21% of mid-run fires in generation 8 fired
with the radio UP — so a peer whose beacons lapsed while its TeamWorld
summaries kept arriving reaches here with an EMPTY missing list.

Falling through would have counted 0 unshared cells over an empty set,
set knowledge=false, and returned a verdict byte-identical to the clean
"the partner already knows everything" suppression — the same
{false,false,0,-1,-1,-1,0,""} row, with nothing in the log able to tell
the two apart. That is the one path where the gate answered "stay"
about a question it never asked. Everything else it cannot evaluate
fails open; so does this, and it says why.
```

### gate-priced-peer-set

**Which peers the gate prices** — in `evaluateReconnectGate`, attached to `std::vector<MissingPeer> live;` (line 80)

```text
The set we are actually pricing: unfinished, addressable peers.

The two ways a peer leaves this list are NOT the same answer, and
collapsing them is how a config fault turns into a silent suppression:

  * FINISHED is a computed no. It will never explore again, so nothing we
    could tell it changes any plan. If that empties the list the gate has
    genuinely decided "not worth it" — it falls through to the knowledge
    count, which is 0 over an empty peer set, and returns a clean
    `dispatch = false` with no `refused`.
  * AN ID OUTSIDE [0,32) is not representable in a `known_by` mask at all,
    so we cannot tell what that peer knows. That is an inability to
    evaluate, and it fails open like every other one. Dropping it quietly
    would let a fleet-identity misconfiguration read as "we are perfectly
    in sync" for the whole run.
```

### gate-peer-position-unknown

**Missing peer with no position** — in `evaluateReconnectGate`, attached to `v.refused  = "peer-position-unknown";` (line 146)

```text
We know someone is missing but not where. Pursuit's own fallback ladder
handles that case (explore on the fallback allowance, then hold in place
and beacon — the last-contact midpoint it used to name was deleted on
2026-09-16); the gate has no basis to price it and must not turn "I do
not know" into "do not go".
```

### gate-two-futures-one-peer

**One peer on both sides** — in `evaluateReconnectGate`, attached to `std::vector<AllocRobot> apart = robots;` (line 158)

```text
Identical vehicle sets, identical world, ONE difference: whether the
missing peers can be given cells they have never heard of. That difference
is the whole measurement, which is why cfg.comms_mask arrives ignored.

ONE PEER ON BOTH SIDES OF THE INEQUALITY (2026-09-18), and it is the peer
`leg` was priced to. The value side used to unmask EVERY live missing peer
while the cost side priced the drive to the nearest one, so the two halves
of C_re described different manoeuvres: the robot paid to fetch one peer
and was credited with the makespan saving of fetching all of them. At N=2
the two coincide and nothing changes; at N>=3 it is a one-directional bias
towards dispatch, and adding a peer with NOTHING to gain from the
reconnection could flip a refusal into a dispatch purely by shortening the
nearest leg. That is a defect against this file's own stated model — see
the leg comment above, "the manoeuvre goes to one of them" — not a policy
choice, so it is corrected here rather than parameterised.

TWO MISMATCHES THIS DOES **NOT** CLOSE, both of which are design questions
about the treatment rather than arithmetic errors, and both of which are
written up in CODE_TODO rather than patched:
  * the node's chase target is the FRESHEST-heard unaccounted peer
    (missingPeerRecord), not the nearest one priced here, so `leg_mm` can
    understate the drive the node actually commits to;
  * a RENDEZVOUS appointment drive is not a trip to a peer at all — it
    reconnects the whole team at an agreed cell — so for that arm the
    single-peer model is the wrong shape in both terms, not just mis-aimed.
Choosing what the gate should price in those two cases changes which
dispatches happen in the arms under test, which is Kalhan's call.
```

### gate-re-vehicle-set

**Building the reconnected vehicle set** — in `evaluateReconnectGate`, attached to `std::vector<AllocRobot> re = apart;` (line 191)

```text
`re` differs from `apart` in exactly one vehicle: the peer we priced the
leg to becomes reachable again. Built from `apart`, not from `robots`,
because `robots` carries whatever in_comms the caller happened to set for
the OTHER missing peers — and in the mid-run trigger's vehicle set that is
false for all of them, which is how the old `robots` solve managed to
unmask everyone.
```

### gate-both-solves-masked

**Both solves run with the mask on** — in `evaluateReconnectGate`, attached to `GlobalAllocator::Config no_cfg = cfg;` (line 202)

```text
BOTH SOLVES NOW RUN WITH THE MASK ON (2026-09-18) and differ only in the
vehicle set. The RE solve used to be `comms_mask = false` over the caller's
unmodified `robots`, and that is the mechanism behind the defect above: the
flag switches masking off wholesale (see GlobalAllocator, which skips the
per-vehicle test entirely when it is clear), so no arrangement of in_comms
bits could have expressed "exactly one peer comes back". Turning the flag
on and restoring one bit expresses it directly, and it also makes the two
solves genuinely identical in every other respect — which is what lets the
difference of their makespans be read as the value of this manoeuvre.

The old comment here claimed `robots` was "everyone reachable, nobody
masked". The first half was not true of the vehicle set the mid-run trigger
actually passes (its missing peers carry in_comms = false); it did not
matter only because the second half made the first irrelevant.
```

### gate-unassigned-structurally-zero

**The structurally zero unassigned column** — in `evaluateReconnectGate`, attached to `v.unassigned = static_cast<int>(no_plan.unassigned.size());` (line 234)

```text
STRUCTURALLY ZERO, and kept only so the column does not change meaning
mid-campaign. The mask exempts any in_comms vehicle from the known-by test,
this robot is always built in_comms, and a vehicle exempt from the test can
take any cell — so no_plan.unassigned cannot be non-empty while self is in
the vehicle set. The column was meant to report comms-mask starvation and
cannot; dead-column status is recorded in CODE_TODO with the rest.
```

## src/separation.cpp

### separation-validate-unconditionally

**Validating radius and age at weight 0** — in `SeparationTerm::configure`, attached to `if (!std::isfinite(cfg.radius_m) || cfg.radius_m <= 0.0) {` (line 16)

```text
Radius and max-age are validated UNCONDITIONALLY, including when the
weight is 0 and they cannot change a decision. They are not inert even
then: the planner logs the peer distance and the eligible-peer count on
every tick of every arm, using this radius and this freshness bound, and
that measurement is the counterfactual a treated arm gets compared
against. Validating them only on the treated side would leave the control
arm measuring separation on a silently different bound.
```

### separation-radius-bounds-2

**Bounds on the separation radius** — in `SeparationTerm::configure`, attached to `if (cfg.radius_m < 1e-3 || cfg.radius_m > 1e6) {` (line 28)

```text
Upper and lower bound, not just positivity. The ramp divides by the radius
in FLOAT: below ~1e-38 the cast underflows to 0 and a candidate exactly on
a teammate evaluates 0/0, which propagates a NaN through std::min and
std::clamp into the logged discount; above ~3e38 it overflows to infinity
and every candidate gets the same discount, so the term is on, costs the
information it takes away, and steers nothing. The bounds are far outside
any real request (1 mm to 1000 km) — their job is to make those two
regimes unreachable, not to express an opinion about plot size.
```

## src/target_scheduler_node.cpp

### target-sched-start-latch

**Latching the schedule origin late** — in `TargetSchedulerNode`, attached to `timer_ = rclcpp::create_timer(` (line 112)

```text
NB: start_time_ is latched on the first tick where the clock is valid,
NOT here. Under use_sim_time the clock reads 0 in the constructor (no
/clock yet); anchoring the schedule to 0 would make the first tick see a
huge elapsed once /clock jumps to the bag's start stamp and release every
target at once. See tick().
```

## src/tree_detector_node.cpp

### node-targets-qos-depth

**Targets topic history depth** — in `TreeDetectorNode`, attached to `const int qos_depth = static_cast<int>(` (line 145)

```text
Latched + deep history so a planner that subscribes after the first
targets were emitted still receives them all. The depth must exceed the
number of targets a run can emit: geometric mode nominates every
vertical structure in the map, not just a preselected handful, so the
old fixed KeepLast(50) could silently drop early targets for
late-joining planners.
```

### node-emitted-trees

**Remembering emitted trees** — in `TreeDetectorNode — declarations`, attached to `struct Emitted {` (line 185)

```text
A tree this node has already published a target for. Outlives the Track it
came from: a track that times out and re-arms must NOT re-emit the same
tree under a fresh id (observed on the map-test-2 bag -- one trunk emitted
twice, ~190 s apart, as 813062754 and 823040963, because a 0.10 m drift in
the centre estimate straddled an id_cell_m boundary). Matching a new track
against this list restores emit-once AND pins the id to the one already in
the planner's queue.
```

### node-build-input

**Converting the map to detector input** — in `TreeDetectorNode — declarations`, attached to `std::vector<SemVoxel> buildInput(const scovox_msgs::msg::ScovoxMap& m) const {` (line 235)

```text
Convert the latest ScovoxMap into the detector's SemVoxel input. Semantic
mode forwards only vegetation-class voxels (the dominant reduction) and
the detector applies the occupancy / confidence gates. Geometric mode
(LiDAR-only maps: semantic records empty, so the veg pre-filter would
silently drop everything) forwards every likely-occupied voxel instead —
the detector's terrain removal + shape gates do the reduction there.
```

### node-one-claim-per-track

**One detection per track per scan** — in `scan`, attached to `std::vector<char> claimed(tracks_.size(), 0);` (line 324)

```text
At most one detection may claim a given track per scan. Without this,
two trunks closer together than match_radius_ both resolved to the SAME
nearest track: `confirm` incremented twice inside a single scan (so
confirm_ticks=2 was satisfied after one scan rather than two in a row,
defeating the consecutive-confirmation requirement) and the second trunk
was silently absorbed into the first track's centre/radius estimate
instead of getting its own track and its own emitted target. Index-
aligned with tracks_; push_back below keeps both in step.
```

### node-new-track-adopts-id

**New tracks adopt emitted ids** — in `scan`, attached to `const int prev = emittedNear(d.center);` (line 358)

```text
New track, created up front so the gate below has one home for both
paths. If this tree was already emitted under an earlier track that
has since timed out, adopt that id and start already `emitted` --
emit-once is a property of the TREE, not of the track that happened
to see it. Note this now also tracks trees that read well-observed
on first sight (the old code only created a track on the needy
branch): under bearing coverage that case cannot arise, and under
map-geometry coverage tracking it is what makes the consecutive-
under-informed streak mean what it says.
```

### node-bearing-verdict

**Bearing-based information verdict** — in `scan`, attached to `float cov = d.angular_coverage;` (line 376)

```text
Default to the detector's own map-geometry score. When a pose is
available, replace the coverage term with the bearings this trunk has
ACTUALLY been viewed from and re-score. The map-geometry proxy cannot
tell "seen from one side" from "circled" once the axis estimate starts
following the observed surface; the bearing history is a direct
measurement of the very thing exploitation improves, so the loop
genuinely closes: circle the tree, the sectors fill, the deficit drops.
```

### node-scan-heartbeat

**The throttled scan heartbeat** — in `scan`, attached to `RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,` (line 414)

```text
Heartbeat so a mode/topic misconfiguration (e.g. semantic mode on a
LiDAR-only map, where every voxel is dropped) is visible instead of the
node just never emitting anything. Logged after the loop because the
under-informed count is the OPERATIONAL verdict (bearing-based when a
pose is available), not the detector's map-geometry one.
```

### node-cell-id

**Position-derived target ids** — in `TreeDetectorNode — declarations`, attached to `std::optional<Eigen::Vector2f> robotXY() {` (line 429)

```text
Deterministic target id from the trunk's map-frame XY, quantised to an
id_cell_m grid. Every robot's detector runs on the SAME fused dscovox map,
so a trunk at a given position hashes to the SAME id fleet-wide — which is
what the planner's id-keyed dedup and team dwell-credit need (a per-node
counter collides: two robots both start at 1 for different trees). Coarse
quantisation (≈ the planner's target_dedup_radius_m) absorbs small per-robot
centre-estimate differences; if two estimates still straddle a cell edge the
ids differ and credit just isn't shared for that trunk (safe degradation to
independent coverage — never a cross-tree mis-merge).
```

## src/vantage_planner.cpp

### vantage-los-voxel-traversal

**Exact line-of-sight voxel traversal** — in `VantagePlanner::lineOfSightClear`, attached to `constexpr float kInf = std::numeric_limits<float>::infinity();` (line 66)

```text
Exact voxel traversal (Amanatides-Woo), NOT a fixed sample spacing.

The reason it has to be exact: NO fixed spacing can guarantee one sample
per crossed voxel. A ray clips the corner of a voxel for an arbitrarily
short length of travel, so the minimum traversal length is 0 no matter the
bearing, while any fixed spacing is positive. Spacing by `res` skips
roughly 25% of the crossed voxels on a diagonal bearing; spacing by
`res / L1(dir)` — the MEAN traversal length, which is what the previous
revision here marched by — narrows the gap but does not close it, and its
comment claimed a guarantee it did not deliver. Worked example at res 0.1
on the (2,1)/sqrt(5) bearing: the ray occupies the voxel at (1,1) for
0.0559 of travel while that spacing is 0.0745, and the occluder inside it
is still reported CLEAR. A miss here sends the exploitation planner to a
viewpoint that cannot see the trunk it was chosen for: in a 270-pair trial
8 of 40 genuinely occluded vantage/trunk pairs came back clear.

So step boundary-to-boundary instead. `t_exit` is the smallest of the three
per-axis crossing parameters, i.e. exactly where the ray leaves the voxel
it is currently in, and the sample is taken at the MIDPOINT of the interval
— strictly interior, so it cannot land on a boundary and read a neighbour.
Every crossed voxel gets exactly one sample and none gets two, which is the
guarantee the old comment asserted. Iterations are the number of voxels the
ray actually crosses, so this is also the cheapest sampling that is
correct.

Voxel boundaries are taken as floor(p / res) * res, in float.

THIS IS NOT EXACTLY THE KEYING MapCache USES, and the comment here asserted
that it was until 2026-09-18. What getVoxel actually does is widen the
float sample position to double and hand it to Bonxai, which keys on
floor(x * inv_resolution) with inv_resolution = 1.0 / resolution held as a
double. Measured, not reasoned: over a scan of seven bearings the float
divide here disagrees with that by a WHOLE VOXEL on ~13 of every 800 nice
round coordinates at res 0.1 — x = 0.7f keys to 6 in the grid and 7 here —
and the float `t_delta` accumulation drifts the later boundaries up by
~1.5e-7 per step on top of that. So this is a real disagreement, not a
rounding curiosity, and the line above overstated the agreement.

IT IS LEFT AS IT IS, DELIBERATELY, and the naive repair is worse. Widening
the entry point to double (`double(from[a]) + d * double(t_begin)`) is the
obvious fix and it is WRONG: the position this loop actually samples is
built in float by Eigen (`from + dir * t`), so a double entry point is a
different point from the one the grid will be asked about. At
from.x = 0.1f that variant puts the ray in voxel 1 while the float sample
lands in voxel 2. Any correct version has to form the entry point in float
— the same arithmetic as the sample — and only then widen for the keying,
and it needs to be checked against a dense ground-truth sweep of the float
ray rather than against these twelve tests, all of which pass under every
variant tried.

DEFERRED ON PURPOSE. The whole lineOfSightClear path is unreachable in a
campaign: run_campaign.sh launches every cell with EXPLOIT=0, so
exploitation_enabled_ is false and all four call sites
(onPeerExploitIntent, doExploitPlan x2, doExploitDwell) are gated off. No
banked or planned result can move on this, which is why it is not worth
taking an unmeasured change into a build. Do not "tidy" it without the
ground-truth sweep.

COVERAGE. Four LoS tests march along +x (LineOfSightClearOnEmptyMap,
LineOfSightBlockedByOccluder, TrunkSurfaceIsCarvedOut,
OccluderJustBeyondTrunkSurfaceBlocks) — the one family of bearings where
every sampling scheme agrees, so they could never have caught the
skipped-voxel defect this traversal replaced. FOUR MORE now do the off-axis
work, and this comment named only the first of them until 2026-09-18:
LineOfSightOffAxisOccluderIsNotSkipped (a crossed voxel must be read),
LineOfSightDoesNotReadVoxelsTheRayMisses (an uncrossed neighbour must not
be), LineOfSightAtFortyFiveDegreesCrossesOnlyTheDiagonal (the exact-corner
case, the only bearing reaching the multi-axis advance below) and
LineOfSightOnANegativeBearingReadsItsFinalVoxel (which is what makes the
midpoint sample load-bearing rather than a style choice). New coverage
still has to be off-axis; it is just no longer true that there is one test.

MUTATION STATUS (2026-09-18, 17 mutants): killed are the tied-axis advance
`<=`->`<` (it stops advancing and the loop never terminates), sampling at
t_enter or t_exit instead of the midpoint, and dropping the one-voxel pad
from `surface`. THIRTEEN SURVIVED. One is a documented equivalent (the
textbook single-axis `break`, see the comment on
LineOfSightAtFortyFiveDegreesCrossesOnlyTheDiagonal). The rest are real
gaps with no test behind them: the own-cell carve-out (`t_begin = 0`
survives), floor vs trunc on negative coordinates, the `occ_stop`
comparison being `>=` rather than `>`, dropping the `stop` clamp on t_exit,
both loop-guard strictnesses, the d == 0 early-out (which every call takes,
since the ray is horizontal by construction), and standoffFor's `lo`
dropping the radius term. Deferred with the keying, same reason.
```

## test/test_candidate_generator.cpp

### test-terrain-z-band-clamp

**Clamping terrain z into the band** — in `CandidateGenerator.TerrainCandidateZIsClampedIntoTheIngestBand`, attached to `TEST(CandidateGenerator, TerrainCandidateZIsClampedIntoTheIngestBand) {` (line 237)

```text
Terrain-snapped candidate z is clamped into the map's ingest band.

The frontier path references the ground search to the CENTROID's own z (a
distant frontier can sit metres above the robot), so ground + z_clearance can
land above the band the map was ingested over — by up to
ground_search_above + z_clearance. A candidate there has its FOV origin
outside the observed volume, where every ray walks un-ingested cells and
scores them as the Beta(1,1) prior, which is maximal: the planner would rank
its own blind spot as the most informative place to go.
```

## test/test_cell_world.cpp

### cellworld-test-exact-resolution

**Why the fixture uses a power-of-two resolution** — in `exactly-representable fixture`, attached to `constexpr float kExactRes = 0.125f;   // 8 columns per 1 m cell` (line 79)

```text
For the tests that compare against MapCache::unknownColumnFraction, the
resolution has to be a power of two. `msg.resolution` is a float, so 0.1f is
really 0.10000000149011612 and Bonxai's inv_resolution is 9.99999985, not
its exact inverse — posToCoord(1.0) is then 9, not 10, and a cell's
"equivalent box" picks up one column belonging to its neighbour. That is a
property of unknownColumnFraction's inclusive-bounds convention rather than a
disagreement about what an observed column is, and testing at 0.125 removes
it so the comparison is about the thing it claims to be about.
```

### cellworld-test-defaults-world-model

**Shipped thresholds assume a saturating map** — in `TEST`, attached to `const CellWorld::Config d;` (line 274)

```text
The thresholds are world-calibrated knobs, in the same family as
done_unknown_fraction (shipped 0.05, overridden to 0.64 by the sim
harness). Both measure "columns with nothing observed in them", and both
have a floor above zero in any world where some columns can never be
observed. There is therefore no universally correct value, and the thing
worth pinning is not the numbers but the world model they encode.

What this test asserts is that the defaults belong to the SAME world model
as the shipped done_unknown_fraction: a map that saturates, where a swept
cell really does approach zero unknown. Raising them to fit one particular
world — which is what the flat-forest sim needs, and what an earlier
version of this file did — silently redefines COVERED for every other
deployment, so it belongs in that world's harness and not here.
```

### cellworld-test-threshold-rounding

**Why the covered test avoids the exact threshold** — in `TEST`, attached to `EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(100, 86)),` (line 385)

```text
Deliberately NOT the exact threshold. 85 of 100 columns is an unknown
fraction of 1.0 - 0.85, which is 0.15000000000000002 in doubles and so
lands on the EXPLORING side of a 0.15 threshold. Behaviour exactly at a
threshold on a continuous measure is not a contract this class offers, and
pinning it in a test would only record which way the rounding happened to
fall for one pair of numbers.
```

### cellworld-test-frontier-veto-fraction

**Frontier veto as a scale-invariant fraction** — in `TEST`, attached to `const CellWorld w = makeWorld();` (line 408)

```text
The regression this replaced. The veto used to be an absolute voxel count
defaulting to 4, which is not a quantity that can be set correctly: it
means something different at every cell size and map resolution, and at
the shipped 10 m cells and 0.1 m voxels a thoroughly swept cell holds tens
of thousands of voxels and hundreds of boundary ones. Every cell was
vetoed for an entire 1800 s run.

So the property under test is scale invariance: the same SHAPE of cell,
measured at two very different sizes, must classify the same way. A count
threshold fails this by construction, which is the point.
```

### cellworld-test-frontier-sentinel

**Unmeasurable frontier must not promote** — in `TEST`, attached to `const CellWorld w = makeWorld();` (line 432)

```text
frontierFraction() reports "cannot measure" as -1.0, and -1.0 sails under
any threshold. A cell with observed columns but no observed voxels cannot
arise from censusFromMap, so this is guarding the seam rather than a
reachable state — which is exactly the kind of check that gets dropped as
impossible and then becomes reachable when a second producer appears.
```

### cellworld-test-hysteresis-band

**Why the hysteresis band holds COVERED** — in `TEST`, attached to `EXPECT_EQ(w.classify(CellStatus::COVERED, obs(100, 75)), CellStatus::COVERED);` (line 462)

```text
Unknown fraction 0.25: above covered_max_unknown (0.15) so it would not be
promoted, but below exploring_min_unknown (0.35) so it is not demoted.
Without this band a cell hovering at the threshold re-commits every tick,
and since every commit resets known_by, the knowledge gate downstream
would never suppress anything.
```

### census-test-shared-column-binning

**Float narrowing and shared column binning** — in `TEST`, attached to `const CellGrid g = makeCellGrid(-15.0f, 5.0f, -15.0f, 5.0f, 5.0f);` (line 916)

```text
The float-narrowing hazard, at a configuration where it actually bites.

A voxel column's low corner is c * resolution in double, but CellGrid works
in float. At resolution 0.05 and a ROI starting at -15, the column at a
cell boundary is -15.000000223517418 in double and exactly -15.0f in
float, so the two floor() to different cells. If the per-cell column count
(the denominator) and the observed-column tally (the numerator) computed
that independently, one cell would be short a column: unknown fractions on
the affected cells would sit slightly below the truth, which reads as a
threshold that needs tuning rather than as a bug.

The configuration matters. At 0.05 m columns over 5 m cells the boundaries
diverge unevenly and one cell in each axis loses a column; at 0.1 m over
1 m cells every boundary diverges the same way, the windows shift as a
block, and the counts come out identical — which is why the other
fixtures here cannot see this and this one can.
```

### census-test-negative-origin-roi

**Float narrowing at a negative-origin ROI** — in `TEST`, attached to `const CellGrid g = makeCellGrid(-15.0f, -13.0f, -15.0f, -13.0f, 1.0f);` (line 947)

```text
The float-narrowing hazard the shared binning helper exists for: at
min_x = -15 and res = 0.2 the boundary column is -15.000000000000002 in
double but exactly -15.0f in float. If the denominator and the numerator
disagreed about that column, the west and south edge cells would report an
unknown fraction slightly below the truth.
```

### sharedhash-test-outage-heal

**Convergence after a link outage heals** — in `TEST`, attached to `CellWorld a = makeWorld();  // self 0` (line 1088)

```text
The unit-level form of the P2 smoke gate. Two robots explore apart with
the link down, diverge, then exchange full state once it heals.

Counts alone cannot score this and the fixture is built to show why: at
the point of maximum divergence both robots have covered exactly two
cells, so every aggregate in cell_census reads identical while the two
worlds disagree about four cells out of nine.
```

### merge-test-third-party-retraction

**A third party cannot undo COVERED** — in `TEST`, attached to `CellWorld w = makeWorld();` (line 1298)

```text
The same shape as the test above, from a robot that is NOT in the mask.
This is the case the plan's table would have applied unconditionally; here
it is a disagreement between two second-hand sources, and the
more-explored ordering settles it so that reordering cannot regress the
census. At N=2 this branch is unreachable, which is why it needs its own
test rather than a smoke run.
```

## test/test_coordination.cpp

### coord-test-exploit-claim-fixture

**The exploitClaim test fixture** — in `exploitClaim`, attached to `Coordination::Claim exploitClaim(const std::string& robot_id,` (line 50)

```text
Peer exploit claim for the rendezvous-barrier tests: the producer is holding
(or driving to) vantage (gx, gy) on `target_id` while its last heartbeat put
it at (rx, ry) and declared `staged` — true only if that peer was in its own
dwell state on (gx, gy) when it published. The pose is still filled in
because MinPos uses it; the barrier does not. `expiry_sec` is on the same
local clock as at()/now.
```

### coord-test-ttl-local-receipt

**Claim TTL uses local receipt time** — in `Coordination.TtlUsesLocalReceiptTimeNotPeerStamp`, attached to `TEST(Coordination, TtlUsesLocalReceiptTimeNotPeerStamp) {` (line 122)

```text
3b. Clock-offset robustness: expiry is based on the LOCAL receipt time, not
    the producer's header.stamp. Field fleets are not clock-synchronised
    (offsets of hours have been observed), so a peer stamp far ahead of the
    local clock must not yield an immortal claim, and one far behind must
    not yield a claim that expires on arrival.
```

### coord-test-claimer-radius

**claimMatching uses the claimer's radius** — in `Coordination.ClaimMatchingUsesTheClaimersRadius`, attached to `TEST(Coordination, ClaimMatchingUsesTheClaimersRadius) {` (line 357)

```text
14. claimMatching sizes each claim's exclusion disc by the radius the
    CLAIMER advertised, not by the receiver's own match radius. The radius
    is phase-dependent (exploration ~8-10 m, one exploit vantage ~0.75 m),
    so evaluating every claim at the receiver's scale made an exploring
    robot veto a whole 8 m disc around a peer that was merely dwelling at a
    trunk.
```

### coord-test-peer-scalars-bounded

**Peer-advertised radius and TTL are bounded** — in `Coordination.HugePeerRadiusIsBoundedByOurOwnExplorationDisc`, attached to `TEST(Coordination, HugePeerRadiusIsBoundedByOurOwnExplorationDisc) {` (line 399)

```text
claimMatching() deliberately evaluates each claim at the radius its CLAIMER
advertised, because the disc size is phase-dependent (~10 m while exploring,
~0.75 m while holding one vantage angle around a trunk). That is correct, but
it means two numbers straight off the wire decide how much of our candidate
set a peer can veto and for how long. Unbounded, either one is a
denial-of-service on our own planner from a single malformed or
version-skewed publish.
```

### coord-test-infinite-peer-radius

**A non-finite peer radius falls back** — in `TEST`, attached to `Coordination c(true, "atlas", /*max_claim_radius_m=*/5.0f);` (line 423)

```text
The specific wire value that defeated the old code: +inf passed the
`radius_m > 0` test, so r2 was infinite and EVERY candidate matched — the
robot yielded every goal to that peer and stopped exploring, silently.
Now it is treated as "peer sent nothing usable" (0), which claimMatching
already handles by falling back to our own match radius.
```

### coord-test-rendezvous-barrier

**The exploit rendezvous barrier query** — in `Coordination.FirstUnstagedExploitPeerNoClaimsIsNullptr`, attached to `TEST(Coordination, FirstUnstagedExploitPeerNoClaimsIsNullptr) {` (line 503)

```text
A robot that has reached its exploitation vantage waits in EXPLOIT_DWELL
until every peer holding an active exploit claim on the SAME target has
DECLARED itself staged on its own claimed vantage, so the whole team dwells
simultaneously. The barrier is up for as long as this query returns non-null,
and the claim it returns is the peer being waited on (logging).
```

### coord-test-staged-flag-contract

**Staging is declared, not inferred from pose** — in `TEST`, attached to `Coordination c(true, "atlas");` (line 537)

```text
The contract: staging is what the PRODUCER declares, and the barrier reads
nothing else. Position used to decide it, and could not tell "on the
vantage" from "arrived in XY, still rotating beside it" (released ~5 s
early, so the team's dwells no longer overlapped) or from "parked at an
approach waypoint", which is an exploit hop but not a vantage at all.
```

### coord-test-barrier-dead-peer

**Barrier release for a dead peer** — in `TEST`, attached to `Coordination c(true, "atlas");` (line 582)

```text
Dead-peer release path. The caller queries this from EXPLOIT_DWELL, where
the PLAN tick -- and therefore prune() -- never runs, so the stale claim of
a peer that died on its way in is still in the table, still unstaged.
There is deliberately NO prune() call in this test: the query must step
over the expired claim itself, otherwise the robot that did arrive dwells
forever.
```

### coord-test-parked-peer-contest

**The selection-time parked-peer contest** — in `Coordination.StagedExploitPeerWinningNoClaimsIsNullptr`, attached to `TEST(Coordination, StagedExploitPeerWinningNoClaimsIsNullptr) {` (line 625)

```text
When the team's synchronised dwells end, both robots re-plan within
milliseconds and both see the SAME single remaining un-dwelled vantage. Their
claims on it cross in the air — the intent heartbeat is only ~1 Hz — so both
drove at the same angle and, in the last field run, collided. This probe
decides the contest at selection time from claims already in the table: a
candidate a parked same-target peer would win under MinPos's total order is
not selectable, so the yield happens before either robot moves. Only PARKED
(staged) peers contest this way; a driving peer contests through its claim
disc alone.
```

### coord-test-contest-self-closer

**Self closer wins without the tiebreak** — in `TEST`, attached to `Coordination c(true, "zulu");` (line 662)

```text
Mirror of the case above — rama is parked on the far side of the trunk and
we are the one standing next to the free angle, so it is ours to take. Our
id is deliberately the LEXICOGRAPHICALLY LARGER one ("zulu" > "rama"): the
tiebreak must not be reached at all when the distances differ, or the
farther robot could steal an angle it is nowhere near.
```

### coord-test-contest-exact-tie

**Exact tie falls through to the id** — in `TEST`, attached to `Coordination self_wins(true, "atlas");` (line 677)

```text
Exactly equidistant, which happens for real whenever the two robots park on
vantages symmetric about the free angle. Candidate at the origin, we at
(4, 0), the parked peer at (-4, 0): powers of two, so both squared distances
are the SAME float and the comparison genuinely falls through to the id
tiebreak rather than being decided by rounding.
```

### coord-test-contest-driving-peer

**A driving peer never blocks the contest** — in `TEST`, attached to `Coordination c(true, "atlas");` (line 704)

```text
Identical geometry to BlocksCandidateWhenPeerCloser — the peer is still the
closer robot — but its heartbeat says staged=false, so it is DRIVING and
this probe ignores it entirely. A moving peer contests only through its
claim disc (claimMatching), because a position contest against a mover would
deny an approaching robot every angle on the ring at once: arriving from far
away it is farther from all of them than a peer already circling the trunk,
and exploitation would serialise instead of running in parallel. The mover's
pose on the wire is up to a heartbeat stale anyway.
```

### coord-test-contest-filters

**Contest ignores other targets and explore claims** — in `TEST`, attached to `c.injectClaimForTest(exploitClaim("rama", 8u, -3.0f, 0.0f, -3.0f, 0.0f,` (line 723)

```text
Both of these peers are staged and both are much closer to the candidate
than we are (they sit on it), so only the target/exploit filters can save
the candidate. Neither peer is on OUR ring: one is parked at a vantage of a
different trunk that happens to be nearby, the other is not exploiting at
all. Contesting an angle of trunk 7 against either would strand us with no
vantage to take on a trunk nobody else is working.
```

### coord-test-exploit-claim-grace

**The exploit-claim grace window** — in `Coordination.ExploitClaimGraceRetainsThroughPrune`, attached to `TEST(Coordination, ExploitClaimGraceRetainsThroughPrune) {` (line 771)

```text
Exploit-claim grace window. The TTL is sized for a 1 Hz heartbeat HEARD at
1 Hz, but the receiver is a single-threaded executor whose EXPLOIT_PLAN
ticks can starve the intent subscription for seconds at a stretch. In a
2-robot sim run the driving winner's claim aged out of the parked loser's
table twice — between plan ticks 7 ms apart — and the loser re-selected the
winner's vantage both times. Grace keeps exploit claims RETAINED (and the
vantage contests closed) one extra window past expiry, while everything
with presence semantics — the dwell barrier, rendezvous counting — stays
on the raw TTL.
```

### coord-test-peer-live-presence

**peerLive shares presence semantics** — in `TEST`, attached to `Coordination c(true, "atlas", 0.0f, /*exploit_claim_grace_sec=*/10.0f);` (line 842)

```text
peerLive is the per-id form of livePeerCount's presence test — it is what
missingPeerRecord() keys the reconnect manoeuvres on (WHICH teammate the
barrier waits on), so it must share the raw-expiry semantics exactly: a
graced-but-expired exploit claim proves table retention, not presence,
and an id never heard is simply not live.
```

### coord-test-barrier-ignores-grace

**The barrier ignores the grace window** — in `TEST`, attached to `Coordination c(true, "atlas", 0.0f, /*exploit_claim_grace_sec=*/10.0f);` (line 901)

```text
THE regression guard for the barrier: grace lengthens how long a claim
can VETO a vantage, and must not lengthen how long a silent peer can
HOLD a dwell barrier. firstUnstagedExploitPeer releases on the raw TTL
exactly as before — a dead teammate frees the dwelling robot in one TTL,
not TTL + grace.
```

## test/test_cost_grid.cpp

### costgrid-unbounded-flood

**Unbounded flood past the grid diagonal** — in `CostGrid.UnboundedFloodExceedsGridDiagonal`, attached to `TEST(CostGrid, UnboundedFloodExceedsGridDiagonal) {` (line 224)

```text
11. An unbounded flood (radius_cap_m <= 0) must reach cells whose PATH cost
    exceeds the grid's straight-line diagonal. The old implementation
    silently clamped the "no bound" case to diag + 1, so a serpentine
    corridor — a walked distance far longer than the diagonal — read as
    unreachable and the exploitation planner rejected drivable vantages.
```

### costgrid-rejected-map-seed

**Rejected map leaves nothing reachable** — in `CostGrid.MalformedMapLeavesNothingReachableIncludingTheSource`, attached to `TEST(CostGrid, MalformedMapLeavesNothingReachableIncludingTheSource) {` (line 263)

```text
A reachability structure may answer "no"; it must never answer "yes" off a
map it has already rejected.

build() marks every cell blocked when data.size() disagrees with the claimed
dims, so the flood has nowhere to go — but the source used to be seeded at
cost 0 unconditionally, which left it the ONLY finite cell in the grid.
reachable(robot_pose) then answered true and reachedCellCount() answered 1
on a map where nothing whatsoever is reachable.
```

### costgrid-seed-on-inflation

**Flooding from an inflated source cell** — in `CostGrid.BlockedSourceWithAFreeNeighbourStillFloods`, attached to `TEST(CostGrid, BlockedSourceWithAFreeNeighbourStillFloods) {` (line 285)

```text
The case that must NOT regress: a robot standing on an inflated cell.

The planning map is inflated by the body radius, so a robot in a dense stand
genuinely stands on a blocked cell. That is routine, not malformed, and the
flood must still start from there — the relaxation refuses to pass through
any OTHER blocked cell, so seeding on inflation cannot route a path through
it. The seed test is therefore "does the flood have anywhere to go", not "is
the source traversable".
```

## test/test_exchange_drain.cpp

### drain-rate-at-decision

**Rate at the moment of decision** — in `rateAtDecision`, attached to `double rateAtDecision(const Counters& counters, int peer, int at) {` (line 202)

```text
The rate the predicate saw at `at`, per the test's own schedule: the
difference over the window that closed there. Used to assert the
precondition test-plan 8 states for both of its cases — a rate of zero at
the moment of decision — so a schedule edit cannot quietly turn 8(a) into
a test of something the rate alone already decides.
```

### drain-finished-absent-peer

**Finished peer that left is not waited for** — in `ExchangePresence.AFinishedPeerThatIsNotHereIsNotWaitedFor`, attached to `TEST(ExchangePresence, AFinishedPeerThatIsNotHereIsNotWaitedFor) {` (line 219)

```text
A FINISHED PEER WITH NO FRESH CONTACT IS NOT WAITED FOR (the A3 regression).

`finished` is sticky and relayed — it says the peer's run is over, not that
the peer is here. §4.1 of the design splits the two questions A3 conflated:
the barrier asks "should I stop waiting?" and is right to count a finished
peer; the exchange asks "is someone here to trade maps with?", which is a
radio statement, and a robot that finished and drove off is not an answer
to it.

cerd announces it is finished and done at t = 0 — it has been to its
meeting and turned for home — and is never heard again; bestla is here,
talks for five seconds, and goes flat. The exchange that is actually
happening is over by the second window. If cerd is counted, it is mute
against its own baseline for the whole hold, and the robot stands on the
cell to the cap waiting for a map that is not coming.

DONE, since §10 item 5 was decided: a finished robot below HOMING is one
still on its way here, and the barrier holds for it before this hold
opens (test_meeting_attendance.cpp). The case this test is for is the one
that left.
```

### drain-finished-present-peer

**Finished peer that is here is waited for** — in `ExchangePresence.AFinishedPeerThatIsHereIsWaitedFor`, attached to `TEST(ExchangePresence, AFinishedPeerThatIsHereIsWaitedFor) {` (line 275)

```text
THE PAIR TO THE TEST ABOVE: A FINISHED PEER THAT IS HERE IS WAITED FOR.

The filter is on presence, and presence only. The over-correction of A3 —
skip finished peers — is wrong in the case generation 32 made the common
one: keepAppointmentOnFinish sends a robot that has finished exploring to
its standing appointment, so the partner on the cell is often finished and
has its whole map still to hand over. Here cerd is finished, direct, and
has delivered nothing since the hold began: the robot must hold for it to
the cap and leave on the unfinished outcome, not release.
```

### drain-relayed-peer

**Relayed peer is waited for** — in `ExchangePresence.ARelayedPeerIsWaitedFor`, attached to `TEST(ExchangePresence, ARelayedPeerIsWaitedFor) {` (line 309)

```text
A RELAYED PEER IS WAITED FOR.

cerd cannot hear atlas at all, but bestla can hear both and says so, so the
model carries cerd as in comms by relay. The emulator forwards serialized
bytes without deserializing them and dscovox credits the robot that SENSED
the voxels, so cerd's counter moves on atlas's side even though nothing
from cerd ever arrives first-hand — gen 32 measured relayed rows as the
most productive exchange channel per row (§2.8). A direct-only filter lets
that stream keep arriving while it calls the exchange finished.

bestla is quiet after its first two seconds; cerd keeps delivering at
2 vox/s — twice R — until s = 40. The release belongs to cerd going flat.
```

### drain-nobody-read

**Nobody read is not everybody drained** — in `ExchangePresence.NobodyReadIsNotEverybodyDrained`, attached to `TEST(ExchangePresence, NobodyReadIsNotEverybodyDrained) {` (line 347)

```text
NOBODY READ IS NOT EVERYBODY DRAINED.

Every `continue` in the loop is a peer the robot could not read, and with
all of them taken a loop seeded `true` would release having examined no
one — at the first full window, on nothing. Here the only peer finished
at t = 0, has not said it is leaving, and has not been heard since: the
robot must not call that a drained exchange. It holds to the cap and
leaves UNFINISHED.

HOW A ROBOT GETS HERE, since §10 item 5 was decided (DESIGN_gen33.md). A
finished partner below HOMING is one still on its way to the meeting, and
the barrier holds for it (holdingForFinishedPeer) — so this hold opens only
once that wait has run out its cap with the partner neither arrived nor
leaving. A no-show, and UNFINISHED is the honest outcome for it. Had the
partner said it was leaving, the next test applies instead.
```

### drain-leaving-peer-unmeasured

**Leaving peer here with unmeasured counters** — in `AllPeersLeaving.ALeavingPeerThatIsHereIsNotSkippedWhenUnmeasured`, attached to `TEST(AllPeersLeaving, ALeavingPeerThatIsHereIsNotSkippedWhenUnmeasured) {` (line 485)

```text
AND WHEN ITS COUNTER CANNOT BE READ. The drain loop does not run on
unmeasurable counters, so `examined` is 0 with bestla standing on the cell;
"nobody here" must be the model's answer, not that zero. Unmeasured holds
to the cap, as DrainUnmeasured says — being told "leaving" by a peer that
is here does not change that.
```

### drain-rose-then-flat

**Counter rose then went flat** — in `BlackoutIsNotADrain.ACounterThatRoseThenWentFlatReleasesDrained`, attached to `TEST(BlackoutIsNotADrain, ACounterThatRoseThenWentFlatReleasesDrained) {` (line 560)

```text
(b) THE COUNTER ROSE, THEN WENT FLAT FOR W. bestla delivers 40 vox/s until
s = 15 and nothing after. The window 0-10 is busy, 10-20 still carries the
tail of it, and 20-30 is the first quiet one: the predicate must release
there, and as kDrained.

Also the rolling window. Each evaluation that does not drain must re-open
the window where it closed, so the next rate is over the next W seconds
and nothing older. A window left open from s = 0 dilutes the burst into an
ever-longer denominator and never gets below R before the cap.
```

### drain-unmeasured-never-drained

**No counters is never drained** — in `DrainUnmeasured.NoCountersIsNeverDrained`, attached to `TEST(DrainUnmeasured, NoCountersIsNeverDrained) {` (line 601)

```text
NO COUNTERS IS NEVER DRAINED.

dscovox publishes the counters only once it is up, and the node's vectors
are empty until then, so a meeting that began earlier has nothing to
difference. That reading is unmeasured, and it must hold to the cap and
leave unfinished rather than release on the absence of evidence. The node
warns on `evaluated && !measurable`, so the reading has to say so too.
```

## test/test_failed_goal_blacklist.cpp

### blacklist-test-shipped-param

**Reading the TTL from shipped config** — in `shipped_param`, attached to `double shipped_param(const std::string& key) {` (line 22)

```text
Read a scalar out of the SHIPPED config instead of restating it as a
literal.

A test that hardcodes 240.0 checks arithmetic the blacklist was never going
to get wrong. What can actually regress is somebody tuning
failed_goal_ttl_sec back down toward the gen-4 value, and against that a
literal is inert: it keeps passing while the claim it stands for stops being
true of anything that runs. Reading the file makes the guard live.

Returns NaN when the key is absent, so the caller can ASSERT on it — a
silent 0.0 default would make every check downstream vacuous in the other
direction, which is the failure mode this helper exists to avoid.
```

### blacklist-prune-out-of-order-stamps

**Prune without assuming age order** — in `FailedGoalBlacklist.PrunesExpiredEntriesOutOfTimestampOrder`, attached to `TEST(FailedGoalBlacklist, PrunesExpiredEntriesOutOfTimestampOrder) {` (line 114)

```text
prune() must not assume insertion order implies age order. Under sim time a
bag restart or a /clock step backwards stamps a fresh entry with an OLDER
timestamp than the one already queued behind it; the old pop-front-until-
fresh loop hit the newer entry first, broke, and left the expired one
blacklisting its goal forever.
```

### blacklist-test-centre-fence

**Two-sided fence for centre drift** — in `TEST`, attached to `EXPECT_TRUE(bl.isNear(p(8.1f, 0), 2.0));    // trailing edge: lost if drifted` (line 168)

```text
Centre never moved. Probed as a two-sided fence, because the obvious probe
is not a probe at all: a point at (11.9, 0) is inside the disc whether the
centre stayed at (10,0), moved to the mean (10.83,0) or jumped to the last
add (11.5,0), so asserting it passes on every build and rules out nothing.
These two bracket the drift instead. Both adds pushed +x, so a centre that
followed them loses the trailing edge and gains the leading one.
```

### blacklist-test-known-answer-replay

**Known-answer replay of re-pick gaps** — in `FailedGoalBlacklist.Seed18GapIsSuppressedAtShippedTtlAndNotAtGen4`, attached to `TEST(FailedGoalBlacklist, Seed18GapIsSuppressedAtShippedTtlAndNotAtGen4) {` (line 224)

```text
KNOWN-ANSWER REPLAY, both sides. mr1_hybrid_seed18 atlas re-picked the same
unreachable site with measured gaps of 210, 203, 224 and 225 s between
failures; each attempt then burned a full 180 s nav budget, 1441 s in total,
and the cell was censored 0.021 above the coverage threshold.

The test asserts BOTH directions on purpose: a one-sided "the new TTL
suppresses it" would also pass on a build where the blacklist suppressed
everything forever, and a guard that cannot fail is not a guard.
```

### blacklist-test-old-ttl-literal

**The deliberate 60 s literal** — in `TEST`, attached to `FailedGoalBlacklist old;` (line 267)

```text
Generation 4 (60 s): expired — this is the defect, reproduced. A literal
on purpose: 60 is a historical fact about a campaign that has already
run, not a value anything still reads.
```

### blacklist-history-outlives-ttl

**Failure history outlives the TTL** — in `FailedGoalBlacklist.CountSurvivesTtlExpirySoRetirementCanFire`, attached to `TEST(FailedGoalBlacklist, CountSurvivesTtlExpirySoRetirementCanFire) {` (line 311)

```text
The test above builds its retirement history from gaps of 210 s and 203 s —
both UNDER the 240 s TTL, so no prune ever lands between two failures at the
same site. That is what let it pass while the mechanism it names was dead in
the field: prune() runs every PLAN tick, and it used to ERASE the record, so
a robot returning to a trap after longer than the TTL found nothing there and
started counting at 1 again. Retirement was therefore reachable only when
every failure fell inside one TTL — the opposite of the "entry ages out while
the robot is busy failing somewhere else" case it was written for.

The field evidence: across at1 + sr3, 64 robot-runs and 177 nav failures, the
logged failure count was 1 on every single one. retire_after=3 never fired.
```

### blacklist-test-sw-corner-trap

**Regression from two real trap sites** — in `FailedGoalBlacklist.Seed2SwCornerTrapAccumulatesAcrossItsRealGaps`, attached to `TEST(FailedGoalBlacklist, Seed2SwCornerTrapAccumulatesAcrossItsRealGaps) {` (line 367)

```text
Regression built from the run that exposed this: at1 seed2, atlas. Two sites
in the SW corner each caught the robot twice, and the gaps are the real ones
(818 s and 804 s) — both far past the 240 s TTL, which is why both were
logged k=1 and neither ever retired. Under the fix the second visit counts as
the second, and a third would close the trap for the run.
```

### blacklist-retired-veto-any-record

**Retired veto survives a nearer site** — in `FailedGoalBlacklist.RetiredVetoSurvivesANearerFreshSite`, attached to `TEST(FailedGoalBlacklist, RetiredVetoSurvivesANearerFreshSite) {` (line 445)

```text
isRetiredNear answers "is any of this suppressed for good?", so a nearer
non-retired record does not overturn it. Under the old nearest-wins form this
query returned false and the planner treated a confirmed trap as an ordinary
suppressed site — in the amnesty ordering, in the WARN, and in the event
field the analysis reads.
```

### blacklist-amnesty-sort-end-to-end

**Amnesty sort, end to end** — in `FailedGoalBlacklist.AmnestySortIsPartitionedAndStable`, attached to `TEST(FailedGoalBlacklist, AmnestySortIsPartitionedAndStable) {` (line 522)

```text
End to end through the same std::stable_sort the planner runs, on indices
into a candidate list, because that is where the two properties interact: the
partition has to hold across the whole list, and equal-key candidates have to
keep the utility order they arrived in (the sort is the last thing standing
between the amnesty pick and an arbitrary one).
```

## test/test_fov_evaluator.cpp

### fov-test-z-band-clip

**Z-band clipping of FOV rays** — in `TEST`, attached to `FovConfig wide;` (line 98)

```text
Rays leaving the [roi_min_z, roi_max_z] band must be clipped so out-of-
band space is never traversed or scored. A tight z-band must yield
strictly fewer scored voxels (and lower total EIG) than an unbounded one,
because the FOV cone contains rays whose vertical pitch exits the band
well before max_range. This guards the dscovox-mode invariant that the
raycast volume matches the GetRegion fetch band (no spurious info gain
from rays pointing into unfetched space above/below the robot).
```

### fov-roi-clip-both-ends

**Both ray ends are ROI-clipped** — in `FovEvaluator.RaysLeavingTheRoiInsideTheDeadZoneScoreNothing`, attached to `TEST(FovEvaluator, RaysLeavingTheRoiInsideTheDeadZoneScoreNothing) {` (line 133)

```text
Both ends of a ray are ROI-clipped, not just the far one. A candidate sitting
on an ROI face and firing outward has its ROI exit at t = 0, i.e. BEFORE the
sensor's min_range: such a ray observes nothing and must contribute nothing.
Previously only the far end was clamped, so the walk started at
position + dir*min_range — outside the box and PAST the clamped far end — and
the iterator ran backwards through voxels the map never ingests, scoring each
as the Beta(1,1) max-uncertainty prior. That inflated info gain precisely at
the ROI boundary, biasing the planner toward the edge of its own region.
```

### fov-roi-clip-parallel-axis

**Out-of-band origin with parallel rays** — in `FovEvaluator.OriginAboveTheBandWithHorizontalRaysScoresNothing`, attached to `TEST(FovEvaluator, OriginAboveTheBandWithHorizontalRaysScoresNothing) {` (line 206)

```text
An origin OUTSIDE the ROI on an axis the ray is PARALLEL to must contribute
nothing. The old exit-only clip skipped any axis with d[i] == 0 entirely, so
it read "parallel to this slab" as "unconstrained by this slab" and walked the
full max_range through space the map never ingested — every cell scoring the
Beta(1,1) max-uncertainty prior, which is maximal.

This is reachable in the shipped terrain-mode config, not a synthetic case:
addFrontierCandidates snaps a candidate to ground + z_clearance with the
ground search referenced to the CENTROID's own z, so a candidate can land up
to (ground_search_above + z_clearance) = 1.3 m above the ingested band. Rays
from a UGV camera are near-horizontal, i.e. d.z ~ 0. The planner would then
score its own blind spot above the band as the most informative place to go.
```

### fov-unknown-voxels-transparent

**Clipping tests need transparent unknowns** — in `File scope`, attached to `namespace {` (line 297)

```text
Five tests above (ZBandClipsRays, RaysLeavingTheRoiInsideTheDeadZone...,
FarEndStillClippedAtTheRoiFace, OriginAboveTheBandWithHorizontalRays...,
NearEndIsClippedAtTheRoiEntryFace) assert that a clipped ray scores nothing
or scores less. Every one of them runs against an EMPTY MapCache, so every
voxel on every unclipped ray is unknown -- and they only measure what they
claim to measure while unknown voxels stay TRANSPARENT. The moment an
unobserved voxel terminates a ray, all five collapse: each ray stops at its
first voxel, the clipped and unclipped counts converge, and the tests either
pass vacuously or fail for a reason that has nothing to do with ROI clipping.

That invariant lives in one clause -- `if (ptr && ptr->p_occ >= occ_stop)` in
fov_evaluator.cpp -- and specifically in the `ptr &&` half. An unobserved
voxel carries the Beta(1,1) prior, an implied p_occ of 0.5, which sits BELOW
the shipped occ_stop of 0.7: so deleting `ptr &&` leaves every shipped run
behaving identically, and the suite would stay green. The failure only appears
once occ_stop drops to 0.5 or below, at which point the evaluator goes blind
at the first voxel of every ray. A guard whose absence is invisible under the
shipped configuration has to be tested under a configuration that is not the
shipped one, which is what UnknownVoxelsStayTransparentBelowTheirPriorPOcc
does.
```

### fov-test-single-ray-geometry

**Single-ray test geometry** — in `File scope`, attached to `namespace {` (line 318)

```text
GEOMETRY. h_rays = v_rays = 1 puts the single ray exactly on the candidate
yaw with zero pitch: precomputeRays() centres the one sample in the FOV cell,
so h_start = -hfov/2 + hfov/2 = 0 and likewise for pitch. With yaw = 0 the
world direction is exactly (1, 0, 0) -- no rounding, no diagonal traversal --
and voxel counts along it are exact rather than approximate.
```

### fov-ray-iterator-half-open

**RayIterator is half-open** — in `File scope`, attached to `constexpr int kLastVisited = kEndCoord - 1;                    // 9` (line 376)

```text
scovox::RayIterator is HALF-OPEN: it visits the start coord and stops BEFORE
the end coord, so the last voxel actually scored is kEndCoord - 1 and the span
is 7 voxels rather than the 8 an inclusive walk would give. This was measured,
not read off the interface, and it has a consequence worth knowing outside
these tests: the voxel at max_range is never scored, so the evaluator's
effective reach is one voxel shorter than cfg.max_range says. At 0.1 m
resolution against a 10 m range that is a 1% bias and not worth chasing --
but it is also the reason an occluder has to be placed at kLastVisited, not
kEndCoord, for a test to see it at all.
```

### fov-test-null-guard-isolated

**Isolating the null-voxel guard** — in `FovOcclusion.UnknownVoxelsStayTransparentBelowTheirPriorPOcc`, attached to `TEST(FovOcclusion, UnknownVoxelsStayTransparentBelowTheirPriorPOcc) {` (line 434)

```text
The `ptr &&` guard, isolated. An unobserved voxel carries the Beta(1,1) prior,
i.e. an implied p_occ of 0.5, so a threshold test written without the null
check stops the ray at its FIRST voxel for any occ_stop <= 0.5. The shipped
0.7 masks this completely, which is why it has to be tested at a threshold
the shipped config never uses.
```

### fov-test-ssmi-no-hit-term

**Isolating the SSMI no-hit term** — in `FovOcclusion.SsmiDropsTheNoHitTermOnlyWhenOccluded`, attached to `TEST(FovOcclusion, SsmiDropsTheNoHitTermOnlyWhenOccluded) {` (line 542)

```text
The SSMI "no hit" term is dropped when the ray was truncated, and this pins it
without reimplementing the estimator.

The trick is to hold the MAP fixed and move only occ_stop, with the occluder
on the LAST voxel of the span. Both configurations then visit exactly the same
voxels in the same order and accumulate bit-identical per-voxel terms; the
only surviving difference is the trailing `if (!occluded)`. Any other
formulation (different map, different occluder position) changes the Beta
parameters or the visit set as well, and the comparison stops isolating the
term it is named after.
```

## test/test_global_allocator.cpp

### alloc-test-cross-perspective-gate

**The cross-perspective determinism gate test** — in `GlobalAllocatorCrossPerspective.IdenticalAllocationFromBothSides`, attached to `TEST(GlobalAllocatorCrossPerspective, IdenticalAllocationFromBothSides) {` (line 58)

```text
THE GATE TEST (plan §4, P3): cross-perspective determinism THROUGH THE WIRE
CODEC. Not two copies of one world — that would only prove solve() is a
function. The worlds are built from opposite viewpoints and reconciled by
exchanging wire messages, so anything that survives the round trip as a
local-only difference (a *_BY_OTHERS provenance byte, a known_by mask, a
local update_id) gets a chance to reach the allocation and change it.

This is the test that fails if someone "improves" the allocator by reading
isFirstHand(), or by preferring cells this robot discovered, or by breaking
a tie on anything that is not (cell id, robot id).
```

### alloc-test-oversize-refused

**Oversized candidate sets are refused, not truncated** — in `GlobalAllocatorBounds.OversizedCandidateSetIsRefusedNotTruncated`, attached to `TEST(GlobalAllocatorBounds, OversizedCandidateSetIsRefusedNotTruncated) {` (line 369)

```text
An oversized candidate set is refused with a reason rather than truncated.
A truncation would be the one failure mode this design cannot survive: two
robots whose candidate sets differ by a single cell would cut the list at
different places and solve different problems while both believing they had
solved the same one.
```

### alloc-hash-blind-channels

**Digest tests prove shared_hash blind channels** — in `AllocHash.EqualProblemsDigestEqual`, attached to `TEST(AllocHash, EqualProblemsDigestEqual) {` (line 453)

```text
These do not test the allocator's answer; they test that the thing that will
be used to decide "were these two robots even solving the same problem" can
actually tell the difference. Every one of them is a channel that
`shared_hash` is blind to, and each test below is the proof of one such
blind channel -- which is the whole claim. (This header used to add "and
that is why robots with equal `shared_hash` disagreed about the peer's focus
23% of the time at N=4"; that figure is withdrawn as unreproducible. These
tests are the evidence, not a campaign statistic.)
```

### alloc-hash-off-frontier-own-assert

**Off-frontier needs its own digest assertion** — in `TEST`, attached to `std::vector<AllocRobot> off = pair2(0, 24);` (line 503)

```text
...one robot off the frontier. Same argument as `finished` directly
above, and it needs its OWN assertion: the two fields are separate, so a
digest that folded in only the first would let two robots disagree about
a peer's mode while agreeing on the key — solving different vehicle sets
under one hash, which is the exact failure the digest exists to expose.
```

### alloc-hash-config-split

**Which Config fields enter the digest** — in `AllocHash.SolverConfigSplitsOnWhatChangesTheProblem`, attached to `TEST(AllocHash, SolverConfigSplitsOnWhatChangesTheProblem) {` (line 611)

```text
Config does not go into the digest as a struct, and does not stay out as a
struct either. The line is whether a field changes the PROBLEM or only the
approach taken to it, and both directions have to be asserted — an
all-in-or-all-out rule is the thing this test replaced (2026-09-18), on both
of the readings the digest is supposed to support:

  "same value, different tours" => a determinism bug in the solver. Only
  sound if fields that change nothing but the tours are EXCLUDED, or the
  finding gets restated as "different problems" and lost.

  "different values" => they were never solving the same problem. Only
  sound if fields that change the problem are INCLUDED, or two genuinely
  different problems agree on the key.
```

## test/test_map_cache.cpp

### mapcache-groundz-contract

**groundZAt role and return contract** — in `groundZAt — local ground elevation under one column`, attached to `namespace {` (line 162)

```text
This is the load-bearing function of terrain_relative_z mode: exploration
candidates, exploitation vantages and (through the vantage z) every
line-of-sight occlusion ray are placed at ground + clearance. It had no test
coverage at all before these.

Contract, from the implementation: coords are voxel corners, so the returned
elevation is the TOP FACE of the top voxel of the ground stack, i.e.
(coord_z + 1) * resolution. At 0.1 m a single ground voxel whose centre is
0.05 sits in coord 0 and reports 0.1.
```

### mapcache-groundz-overhang

**An overhang with no ground reads as ground** — in `MapCacheGroundZ.AnOverhangWithNoGroundBelowIsReportedAsGround`, attached to `TEST(MapCacheGroundZ, AnOverhangWithNoGroundBelowIsReportedAsGround) {` (line 199)

```text
CHARACTERISATION, not an endorsement. With no ground voxel in the column but
an overhang inside the window (a fallen log, a ledge, a low branch, or simply
ground that the sensor has not seen yet under something it has), groundZAt
returns the overhang's top face — a plausible, FINITE, wrong answer. Every
caller's only defence is std::isfinite, which this passes.

The consequence in terrain mode: the vantage is placed z_clearance above a
branch, and lineOfSightClear marches its occlusion ray at that height. With
use_planning_map false, that LoS test is the only real vantage filter, so a
wrong-but-finite ground silently produces a confidently-wrong capture pose.
The dwell re-confirms LoS from the settled pose, which limits the damage to a
wasted vantage rather than a false success, but nothing detects the cause.
If this behaviour is ever changed (e.g. require a supporting stack, or return
NaN when the column below the hit is unobserved rather than free), this test
is the one that should fail and be updated deliberately.
```

### mapcache-wire-resolution-reject

**Rejecting a non-finite wire resolution** — in `TEST`, attached to `scovox_msgs::msg::ScovoxMap good;` (line 302)

```text
A positive-but-infinite msg.resolution passed the old `> 0.0f` guard
straight into the Grid, making inv_resolution zero and every posToCoord a
float->int32 cast of a non-finite value: UB that shows up as garbage
coordinates, not a crash. Reject the MESSAGE (this arrives over the wire
mid-run) rather than throwing, so one malformed publish cannot take the
planner down in the field.
```

## test/test_meeting_attendance.cpp

### attendance-keeper-whole-run

**A keeper's run and the early DONE** — in `AnnouncedMode.AKeeperSaysDoneOnlyOnceItStopsKeepingTheAppointment`, attached to `TEST(AnnouncedMode, AKeeperSaysDoneOnlyOnceItStopsKeepingTheAppointment) {` (line 122)

```text
THE KEEPER'S WHOLE RUN, tick by tick. The defect this generation fixes is
the second line: a robot that saturates with an appointment standing used
to say DONE there, before the drive, so "will it take part?" answered no
for a robot on its way to the meeting.
```

## test/test_metrics_logger.cpp

### mlog-test-plan-column-order

**Distinct values catch swapped plan columns** — in `MetricsLoggerSchema.EachPlanColumnCarriesItsOwnValue`, attached to `TEST(MetricsLoggerSchema, EachPlanColumnCarriesItsOwnValue) {` (line 86)

```text
The ordering check a column-count test cannot do.

Every new field gets a DISTINCT value, so swapping any two of them in the
writer — plan_rej_map for plan_rej_unreach, say — fails here. That specific
swap is the one that matters: it mislabels the cause of a starvation, which
is the entire quantity these columns were added to record, and both columns
are ints so nothing else would ever flag it.
```

### mlog-test-plan-minus-one

**Unattempted plan columns write -1** — in `MetricsLoggerSchema.UnattemptedPlanColumnsWriteMinusOneNotZero`, attached to `TEST(MetricsLoggerSchema, UnattemptedPlanColumnsWriteMinusOneNotZero) {` (line 122)

```text
-1 must survive to the file as -1.

These columns are read as "no planning attempt has happened yet". If the
default were ever changed to 0, or clamped anywhere on the way out, a row
from before the first plan would be indistinguishable from a row recording
a genuine zero-rejection tick. That ambiguity is exactly what made
rejected_by_minpos useless for a whole campaign.
```

### mlog-test-append-only-tail

**New CSV columns are appended at the end** — in `MetricsLoggerSchema.NewColumnsAreAppendedAtTheEnd`, attached to `TEST(MetricsLoggerSchema, NewColumnsAreAppendedAtTheEnd) {` (line 165)

```text
The new block sits at the right-hand end, which is the schema rule the
header comment states: readers outside this tree may resolve positionally,
and an old file read against a new schema must stay aligned up to the point
where it simply runs out of columns.

R5 added a block AFTER the plan_* block, so the assertion is now on the
last ten names rather than the last seven, and the plan_* block is pinned
in place by its own position rather than by being last. That is the point:
this test is what makes "appended, never inserted" a checked property
instead of a comment, and updating it is the cost of every append.

v8 appended `plan_rej_visited` — eleven now. That column belongs beside
`plan_rej_blacklist` by meaning and is at the far end instead, which is
exactly the pressure this test exists to resist: the tidy edit shifts eight
columns under every positional reader of every banked run, and nothing else
in the tree would notice.
```

### mlog-test-sanitize-empty

**The only test of sanitizeField on empty input** — in `TEST`, attached to `EXPECT_EQ(v["pursue_peer"], "")` (line 202)

```text
This line is also the ONLY reachable test of sanitizeField's empty input.
The function is TU-local, so it can only be exercised through the writer,
and F10 deleted an `if (s.empty()) return ""` fast path whose comment
claimed it wrote "-". It never did — the loop returns "" for an empty
string anyway — but the comment was what a reader would have believed.
Asserting "" here is what keeps the deletion a refactor.
```

## test/test_plan_map_query.cpp

### planmap-short-data-buffer

**Grid metadata that overstates data size** — in `PlanMapQuery.ShortDataBufferIsNoDataNotAnOverread`, attached to `TEST(PlanMapQuery, ShortDataBufferIsNoDataNotAnOverread) {` (line 137)

```text
A grid whose METADATA claims more cells than data[] actually holds.

nav_msgs ties nothing together: info.width/info.height are a separate claim
from data.size(), and every bounds test in this file is written against the
former. A publisher that fills in the header and then sends a short (or
empty) vector therefore produces a grid that passes every index check and
still reads off the end — a 1140x1140 map carrying 1140 bytes segfaulted the
planner under AddressSanitizer.

Both functions must degrade to their existing "cannot be measured" answer
rather than to a fabricated one. That direction matters for
unknownFractionInRoi in particular: its output feeds the DONE criterion, so
inventing a fraction would end a run, while -1.0 is already read as "no
reading this tick".
```

## test/test_planner_util.cpp

### util-reconnect-mode-parse

**Parsing the reconnect mode string** — in `PlannerUtil.ReconnectModeFromString`, attached to `TEST(PlannerUtil, ReconnectModeFromString) {` (line 100)

```text
The wire strings are what the yaml/launch pass; anything else must fall back
to the legacy behaviour, never crash or invent a mode. Matching is
case-insensitive (a hand-typed "Hybrid" must not silently run legacy
rendezvous), and the `known` flag is what the node's startup warning keys
on, so it must be false exactly when the fallback was NOT asked for.
```

### util-pursuit-ceiling-beats-floor

**The pursuit ceiling wins over the floor** — in `PlannerUtil.PursuitBudgetCeilingBeatsFloor`, attached to `TEST(PlannerUtil, PursuitBudgetCeilingBeatsFloor) {` (line 163)

```text
The ceiling wins over the floor. min_sec is the nav-family floor
(nav_min_timeout_sec, 30 s in the yaml) and max_sec the pursuit-family
ceiling — nothing orders them, and a short-chase A/B like
pursuit_budget_max_sec:=15 is legitimate config. The naive
clamp(raw, 30, 15) is UB (lo > hi) whose libstdc++ artifact returned the
FLOOR — a budget above the "hard ceiling" the waiting teammate relies on.
The ceiling must hold from both directions: raw below the floor and raw
above the ceiling.
```

### util-pursuit-degenerate-inputs

**Degenerate pursuit budget inputs** — in `PlannerUtil.PursuitBudgetDegenerateInputs`, attached to `TEST(PlannerUtil, PursuitBudgetDegenerateInputs) {` (line 180)

```text
Degenerate inputs must stay inside [0, max_sec]: a negative staleness
(clock skew between the record stamp and now on the same local clock is
impossible, but a caller bug must not inflate the budget past full
freshness) and a zero/negative speed estimate (guarded to 1e-3, so the raw
term explodes and the ceiling absorbs it — same guard navBudgetSec tests).
```

### util-alloc-ttl-unbounded

**Unbounded peer-position TTL admits everything** — in `PlannerUtil.AllocPeerPositionUnboundedAdmitsEverything`, attached to `TEST(PlannerUtil, AllocPeerPositionUnboundedAdmitsEverything) {` (line 194)

```text
max_age <= 0 is UNBOUNDED. This is the case that matters most: it is the
default the parameter ships with, and it is what makes the TTL binary
reproduce the pre-TTL planner, so one build can run both arms of a campaign.
Nothing about the age may change the answer here — not a huge age, not the
"no position held" sentinel, not a negative bound.
```

### util-alloc-ttl-unknown-position

**No position held is never fresh** — in `PlannerUtil.AllocPeerPositionUnknownIsNotFresh`, attached to `TEST(PlannerUtil, AllocPeerPositionUnknownIsNotFresh) {` (line 218)

```text
TeamModel::positionAgeSec() returns a NEGATIVE age for "no position held".
Under a live TTL that must read as NOT fresh. The caller also tests
have_position, so this is belt-and-braces — but the failure it guards is
silent: "we have never located this peer" coming out identical to "we heard
from it a moment ago" would hand the whole map to a robot we cannot find.
```

### util-alloc-ttl-nan-drops

**A NaN age or bound drops the peer** — in `PlannerUtil.AllocPeerPositionNonFiniteIsNotFresh`, attached to `TEST(PlannerUtil, AllocPeerPositionNonFiniteIsNotFresh) {` (line 236)

```text
A non-finite input drops the peer instead of admitting it. The node refuses
a non-finite parameter at load, so this should be unreachable from the
harness; it is asserted because the NaN answer falls out of comparison
semantics rather than from any written branch, and the safe direction (drop)
and the dangerous one (admit) are one operator apart.
```

### util-nav-budget-finite-deadline

**The NAVIGATE timeout must always fire** — in `PlannerUtil.NavBudgetIsAlwaysAFiniteFiringDeadline`, attached to `TEST(PlannerUtil, NavBudgetIsAlwaysAFiniteFiringDeadline) {` (line 261)

```text
A watchdog that cannot fire is worse than no watchdog.

navBudgetSec is the NAVIGATE timeout. Both properties asserted here are
guarantees rather than incidental behaviour, and both used to fail:

 - A non-finite input survived the clamp. std::clamp is written as
   `v < lo ? lo : hi < v ? hi : v`; both comparisons answer false against
   NaN, so the NaN came straight back out. Every later `elapsed > budget`
   test is then false too — the timeout never fires, the robot sits on a
   dead goal, and the cell runs to max_steps. In the campaign record that is
   indistinguishable from a genuinely slow cell, so a censored run is scored
   as a completed one.
 - Nothing orders min_sec against max_sec (they are separate parameters from
   unrelated families), and std::clamp with lo > hi is undefined behaviour.
```

### agreed-occurrence-overshoot-bound

**Why the overshoot bound is conditional** — in `TEST`, attached to `for (int i = 0; i < 400; ++i) {` (line 367)

```text
The overshoot bound is conditional on purpose. Before t_meet there is no
earlier occurrence to choose — k = 0 is the first meeting the team agreed
to — so the gap to the floor is whatever the agreement made it, and
asserting "within one interval" there would be asserting that the function
invents a meeting between now and the first agreed one. That is exactly
what the countdown did.
```

### agreed-occurrence-arming-spread

**Robots on one agreement differ by intervals** — in `TEST`, attached to `const long long t_meet = 130000, interval = 30000;` (line 393)

```text
THE WHOLE POINT, as a test. The ts4 cells measure ~1.2 s of skew between
robots' mission clocks, and the generation-18 measurement that killed the
first attempt at an agreed time was an arming spread of 16.1 / 52.5 /
67.4 s. Feed that spread in against one committed triple: the three robots
may choose different k, but every pair differs by a whole interval, so the
early ones are standing at the agreed cell when the late one arrives. Under
the countdown the differences were 36.4 s and 14.9 s — not multiples of
anything, and never resolvable.
```

### arrival-shortfall-no-fork

**A team that can attend does not fork** — in `TEST`, attached to `const long long t_meet = 130'000, interval = 30'000;` (line 503)

```text
The composed property, which is the only one that matters at the call site:
feed one committed pair and a spread of drives, and every robot inside its
budget must land on the SAME instant — not merely on the same lattice.

Drives of 10-60 s against a 60 s budget and a 30 s lattice: all six robots
keep t+130 s. Under the generation-19 rule the same six split across three
occurrences.
```

### flicker-dwell-window-ownership

**The flicker dwell window and its failures** — in `FlickerDwell.TheFirstEligibleTickArmsAndDoesNotFire`, attached to `TEST(FlickerDwell, TheFirstEligibleTickArmsAndDoesNotFire) {` (line 536)

```text
Three sites in the node act on "the team came back": the manoeuvre barrier,
the appointment supersede, and the rendezvous_spent_ latch release. All three
read that fact from a claim table with a 5 s liveness TTL, where ONE packet
arriving is enough to make the team look whole for a reading — so all three
have to see it HOLD. Generation 23 moved the holding rule out of the node and
into these two functions, which is the only way it can be tested at all: the
node defines main(), every gtest target links gtest_main, so nothing there is
callable from here.

dwellConfirmed owns a window; dwellHeld reads one. The split is not cosmetic.
The supersede site and the latch release share ONE window, written once per
heartbeat and read from doPlan, because they ask a single question and two
windows over one predicate is how "the outage is over" gets two answers. A
second caller into dwellConfirmed would advance or disarm the pair, which
would mean that asking the question changed it.

THE TWO FAILURES THESE TESTS EXIST FOR, both of which ship silently:

  * A dwell that fires on the arming tick. It reads as a guard, has a
    parameter, appears in the logs, and filters nothing.
  * A FROZEN window — dwellConfirmed called from behind a branch that is
    untaken for minutes. An un-ticked window does not decay; `armed` stays
    true with a stale `since_sec`, and the first sample after the gap sees
    `now - since` far past the confirm time and fires on ONE reading. Same
    visible symptom as no guard at all, but harder to see in the source.
```

## test/test_pursuit_predictor.cpp

### pursuit-fixture-world5

**The 5x5 pursuit test world** — in `world5`, attached to `CellWorld world5(int self = 0) {` (line 23)

```text
A 5x5 world of 10 m cells over [-25, 25]^2. Row-major ids, so the bottom
row is 0..4 with centres (-20,-20), (-10,-20), (0,-20), (10,-20), (20,-20)
— a straight line of 10 m hops, which is what makes every number below
hand-computable.

No cell edges are set, so CellWorld::distance reports unreachable and
GlobalAllocator::costMm falls back to centroid distance. That fallback is
the documented behaviour (global_allocator.hpp) and it is what the predictor
inherits; a test that quietly depended on a roadmap would be testing a
configuration the node does not always have.
```

### pursuit-fixture-exact-chain

**An exactly computable chain config** — in `exact`, attached to `PursuitPredictor::Config exact() {` (line 42)

```text
A chain whose arithmetic is exact: 1 m/s over 10 m legs with no dwell is a
10 s leg, and a 5 s step makes P(go) exactly 1/2. Off-route drain is
switched off outright (half-life 0, which the Config defines as "the hazard
is off") so the distribution is a clean binomial and any deviation is the
model, not the hazard. A merely LARGE half-life would not do: at 1e9 s each
step still loses 3e-9 of the mass, which is a thousand times the tolerance
these tests need in order to be checking the binomial at all.
```

### pursuit-intercept-gate-test

**The hand-computed intercept gate test** — in `PursuitPredictorIntercept.MeetsInTheMiddleAgainstAHandComputedChain`, attached to `TEST(PursuitPredictorIntercept, MeetsInTheMiddleAgainstAHandComputedChain) {` (line 227)

```text
THE GATE TEST (plan §4, P6): argmax intercept against a hand-computed case.

The peer is at cell 0 driving 0->4; I am at cell 4. My drive to each cell is
40/30/20/10/0 m, so at 1 m/s I would arrive after 8/6/4/2/0 steps of 5 s.
Reading the binomial rows at those steps:

    cell 0 @ 8 steps: 2^-8            = 0.0039
    cell 1 @ 6 steps: 0.09375
    cell 2 @ 4 steps: 6/16            = 0.375   <-- the argmax
    cell 3 @ 2 steps: 0
    cell 4 @ 0 steps: 0

The answer is cell 2: not where the peer WAS (cell 0, which is what the
legacy trail would chase) and not where I already am. That gap is the whole
point of §3.7, and this test is what would fail if the horizon reverted to
the record age alone.
```

### pursuit-age-moves-intercept

**Record age moves the intercept forward** — in `PursuitPredictorIntercept.AgeAloneMovesTheInterceptForward`, attached to `TEST(PursuitPredictorIntercept, AgeAloneMovesTheInterceptForward) {` (line 285)

```text
Staleness is inside the horizon, so a record that has aged moves the
intercept forward on its own, with no change to where either robot is. Both
robots stand on cell 0 here; the only difference between the two calls is
how long ago the tour was heard.

With the hazard off the tour's last node absorbs, so the aged intercept runs
all the way to the tail (30 s of staleness is six steps of a chain whose
P(go) is 1/2, and the mass has nowhere else to end up). That is the model
being honest rather than a defect: a tour heard long enough ago and never
contradicted really does say "it finished". In a run the half-life drains
that same mass into O and `min_probability` returns the chase to the trail —
which is what AStaleRecordRefusesButStillReports covers.
```

### pursuit-tie-break-order

**The intercept tie-break order** — in `PursuitPredictorIntercept.TiesBreakOnSoonerThenEarlierIndex`, attached to `TEST(PursuitPredictorIntercept, TiesBreakOnSoonerThenEarlierIndex) {` (line 319)

```text
The tie-break is total and its FIRST rule is "sooner arrival". Both
surviving candidates here score exactly zero — the mass cannot reach either
in the steps available — so the only thing separating them is the rule.
The duplicated tail cell then exercises the second rule (lower tour index)
at an identical horizon.
```

### pursuit-no-tour-named-refusal

**No tour on record is a named refusal** — in `PursuitPredictorDegradation.NoTourRefusesAndSaysWhy`, attached to `TEST(PursuitPredictorDegradation, NoTourRefusesAndSaysWhy) {` (line 347)

```text
§3.7's hard requirement: with no tour on record, pursuit is exactly what it
was before this file existed. The refusal must be a NAMED one — "the model
had nothing to say" and "the model was never asked" are different runs, and
one empty answer for both would make an arm that never predicted look like
one that predicted badly.
```

### pursuit-dwell-pulls-intercept-back

**Why dwell must change the intercept** — in `PursuitPredictorDwell.WorkingTheCellsPullsTheInterceptBack`, attached to `TEST(PursuitPredictorDwell, WorkingTheCellsPullsTheInterceptBack) {` (line 510)

```text
A peer that works its cells advances more slowly than one that drives
through them, and the intercept has to move back to meet it. Zero dwell is
the setting that predicts the peer far ahead of where it is (see the
Config comment), so the two must give different answers or the parameter is
decorative.
```

## test/test_reconnect_gate.cpp

### gate-tests-paired-controls

**Reconnect gate tests and their controls** — in `File scope`, attached to `#include <gtest/gtest.h>` (line 1)

```text
Tests for the economic reconnection gate (§3.6).

The gate's risk is not that it crashes; it is that it is quietly wrong in
ONE DIRECTION and nothing notices. A gate stuck closed suppresses every
reconnection and presents as "the info gate found nothing worth fetching".
A gate stuck open is today's silence clock with extra logging. So every
suppression here is paired with a control that fires on the same fixture
with one lever moved, and the two failure directions the plan names are
tested as such rather than assumed away.
```

### gate-fixture-production-shape

**The production-shape vehicle list fixture** — in `pair2Apart`, attached to `std::vector<AllocRobot> pair2Apart(int cell_self, int cell_peer) {` (line 54)

```text
PRODUCTION SHAPE: the peer the caller reports missing is also flagged
out-of-comms in the vehicle list.

`pair2` marks both vehicles in_comms=true, which is a pairing the node
cannot produce — explo_planner_node builds `missing` by scanning that very
list for `!r.in_comms`, so a missing peer is out-of-comms BY CONSTRUCTION.
Every test in this file used pair2, and the consequence was that
GlobalAllocator's comms mask — which only ever restricts a vehicle whose
`in_comms` is false — was a no-op across the whole suite. The gate's entire
value half turns on that mask (it is what makes C_no "the cost of finishing
without telling them"), so the suite was blind to it: flipping
`re_cfg.comms_mask` in the production path would not have failed a test.
```

### gate-fixture-world-with-work

**The worldWithWork fixture** — in `worldWithWork`, attached to `CellWorld worldWithWork(int n) {` (line 180)

```text
`n` EXPLORING cells, laid in from the far corner back towards the origin so
the work is always the part of the map furthest from robot 0 at cell 0.
Every one is first-hand, so under the no-comms mask the peer may take NONE
of them and the whole tour falls to me — which is what makes C_no the cost
of staying apart rather than an artefact of where the peer happens to be.
```

### gate-value-lever-is-work

**Why the value lever is work** — in `ValueGate.EnoughWorkToDivideIsWorthTheTrip`, attached to `TEST(ValueGate, EnoughWorkToDivideIsWorthTheTrip) {` (line 194)

```text
The lever here is the AMOUNT OF WORK, with the peer nailed to cell 4 so the
leg is the same 40 m in both directions. That is deliberate: peer distance
is a confounded lever, because moving the peer changes what it can reach as
well as what it costs to reach it — in this fixture a peer parked further
away up the right-hand side is CLOSER to the work and therefore worth MORE,
not less. Work is the clean one.
```

### gate-knowledge-and-value-are-and

**The knowledge and value gates are AND** — in `TEST`, attached to `CellWorld w = world5(0);` (line 229)

```text
The two gates are AND, not OR. Here there is genuinely something to share
(a COVERED cell the peer has never heard of) and genuinely nothing to gain
by sharing it (no work left to divide). Also pins the leg's arithmetic:
with both makespans zero, C_re IS the leg, at two different distances, so
the drive enters the cost one-for-one and not scaled or dropped.
```

### gate-empty-missing-fails-open-2

**An empty missing set fails open** — in `TEST`, attached to `CellWorld w = world5(0);` (line 370)

```text
This test used to assert the opposite — dispatch=false, refused="" — on
the reasoning that "a trigger that reaches the gate with an empty peer set
has a bug upstream", so answering "go anyway" would hide it.

The premise was wrong twice. It is not a bug: the trigger reads the
coordination beacon and this list is built from TeamModel::inComms, two
different topics with different TTLs that are EXPECTED to disagree, so an
empty set here is a normal outcome. And the old verdict hid it far better
than a dispatch would have — {dispatch=false, refused="", unshared=0} is
byte-identical to the clean "the partner already knows everything"
suppression, so the log could not distinguish "we decided to stay" from
"we never identified anyone to ask about".
```

### gate-finished-peers-clean-no

**All-finished peers are a computed no** — in `TEST`, attached to `CellWorld w = world5(0);` (line 390)

```text
The other way the priced set empties, and it must NOT be confused with the
one above: a FINISHED peer will never explore again, so nothing we could
tell it changes any plan. That is a computed no, and it keeps the empty
`refused` that marks a real decision. Asserted here because the fail-open
added for the empty-`missing` case sits directly upstream of this path and
would swallow it if it were written one line too broadly.
```

### gate-dispatch-reachable-apart

**Dispatch is reachable in production shape** — in `TEST`, attached to `CellWorld w = world5(0);` (line 411)

```text
THE DIRECTION THE LIVE SMOKE NEVER EXERCISED. A p4 smoke cell produced 11
gate evaluations and suppressed all 11, which is consistent both with a
discriminating gate and with a gate wedged shut — and no test could tell
the two apart, because every fixture in this file was in_comms=true, where
the mask cannot bite and C_no cannot rise. So this asserts the reachability
of "go", on the shape the node actually builds.

The fixture has to make BOTH terms favourable, and the first draft of this
test got both wrong in an instructive way. Peer at the opposite corner
(cell 24) makes the leg 56.6 m across a 50 m grid, which alone exceeded the
whole coordination benefit — the gate correctly said stay, and the test was
asserting the gate was broken for getting it right. And a backlog packed
into self's own corner is work the distant peer would not have taken even
unmasked, so there was no benefit to buy.

So: peer ADJACENT (cell 1, one 10 m hop, a cheap leg) and the backlog
spread across the whole grid, where splitting it two ways genuinely halves
the makespan and the peer has a bit for none of it.
```

### gate-comms-mask-mechanism

**Observing the comms mask in the gate** — in `TEST`, attached to `CellWorld w = world5(0);` (line 446)

```text
Pins the mechanism, and the comparison has to be chosen carefully to see
it. The obvious test — same fixture, flip the peer's in_comms, expect C_no
to move — cannot work and its failure is not a defect: the gate copies
`robots` into `apart` and sets in_comms=false on every live missing peer
ITSELF, so the caller's flag never reaches the C_no solve. That is the
right design (the caller cannot accidentally price an unmasked future),
but it means the mask has to be observed against the OTHER solve.

So compare the two futures the gate actually builds. c_re_mm carries the
leg; net it off and what remains is makespan(re_plan), solved over the same
world and the same vehicles with the mask OFF. Against a backlog the peer
holds no bit for, barring it must make the apart-makespan strictly larger.
If these come out equal, no_cfg.comms_mask is not reaching the solver and
the entire value half is arithmetic over two identical problems — the
mutation this suite previously could not catch.
```

## test/test_scoring.cpp

### scoring-entropy-non-finite

**Entropy of a non-finite probability** — in `Scoring.EntropyIsZeroForNonFiniteProbability`, attached to `TEST(Scoring, EntropyIsZeroForNonFiniteProbability) {` (line 125)

```text
entropy() must return 0 for a non-finite p_occ. The saturation guard was
written as `p < eps || p > 1-eps`, and every comparison against NaN is
false, so a NaN slipped through and returned NaN — which then poisoned the
accumulated ray score and mean_entropy in the metrics CSV for the rest of
the run.
```

## test/test_separation.cpp

### sep-weight0-keeps-radius-maxage

**Weight-0 configure keeps radius and max age** — in `TEST`, attached to `SeparationTerm s;` (line 63)

```text
The control arm's DIAGNOSTICS are measured on these two numbers, in every
arm, and they are the counterfactual a treated arm is read against. If a
weight-0 configure quietly reverted them to the defaults, an off arm asked
for a 25 m radius would have reported its peer distances against 20 m and
the two arms' columns would not have been comparable — a mismatch nothing
downstream could see, because both arms would print the requested 25 in
their manifests.
```

### sep-radius-float-bounds

**Why radius needs more than positivity** — in `TEST`, attached to `for (double r : {1e-40, 1e-30, 1e-4, 1e7, 1e39, 1e300}) {` (line 116)

```text
Positivity is not enough. The ramp divides by the radius in float: a radius
that underflows the cast to 0 makes a candidate sitting exactly on a
teammate evaluate 0/0 and log a NaN discount, and one that overflows it to
infinity gives every candidate the same discount, so the term takes the
information away and steers nothing. Both pass a bare `> 0` check and both
look like a working term in the manifest.
```

### sep-radius-not-hardcoded

**Radius honoured, not hard-coded** — in `TEST`, attached to `const auto peer = anchorsAt({{0.0f, 0.0f}});` (line 260)

```text
Added after a mutant that replaced cfg_.radius_m with a literal 20.0f
survived the whole suite: every other case here happens to use the default
radius, so a knob that was parsed, validated, logged into the manifest and
then ignored would have looked exactly like a working one. That specific
failure — a configured value the binary does not actually use — is the one
this project keeps paying for, so it gets its own case.
```

### sep-max-age-in-eligible-anchor

**Why the freshness bound lives in SeparationTerm** — in `SeparationEligible.HonoursTheConfiguredBoundAndIsNotHardCoded`, attached to `TEST(SeparationEligible, HonoursTheConfiguredBoundAndIsNotHardCoded) {` (line 432)

```text
max_age_sec used to be consumed entirely in the planner node, in a private
method no test could reach. A review of this file found that replacing the
comparison there with a constant passed the whole suite — the knob would have
been parsed, range-checked, written into every run manifest, and then not
read, which is a failure this project has already paid for more than once.
The comparison moved in here so these tests can hold it.
```

## test/test_team_model.cpp

### gossip-resend-does-not-refresh

**A re-sent relay reading must age** — in `TEST`, attached to `TeamModel r = makeModel();` (line 397)

```text
And a relay that keeps re-sending the same reading must not keep
refreshing it. Note what "the same reading re-sent" looks like on the
wire: the entry for cerd is unchanged at 190, but the SENDER'S OWN entry
advances every publish, because it is the sender's publish time. So the
age grows message by message and `at` stays put — which is the point.
(Constructing this with a frozen sender entry instead would be a
duplicated message, not a re-send; see the note in observe().)
```

### team-position-vs-known-age

**Position age versus last-known age** — in `TeamModelPositionAge.NegativeUntilAPositionArrives`, attached to `TEST(TeamModelPositionAge, NegativeUntilAPositionArrives) {` (line 471)

```text
Rule 3 of the file header, made testable. lastKnownAgeSec() answers "when
did we last learn ANYTHING about this robot"; positionAgeSec() answers "how
old is the pose we hold for it". Both refresh paths in observe() advance
last_known_sec BEFORE testing whether the message carried a position at all,
so the two genuinely come apart, and the consumer that steers on a peer pose
— a chase, the separation term — is the one that would be wrong about it.
```

### r1-packet-stamp-timing

**R1 timings use packet stamps** — in `TeamModelR1.AcquisitionIsMeasuredFromTheFirstOneWayPacket`, attached to `TEST(TeamModelR1, AcquisitionIsMeasuredFromTheFirstOneWayPacket) {` (line 654)

```text
Timing the detector, never changing it: every assertion below is about WHEN
a transition was reported, and none of them is about whether `direct` was
right. The one design property they exist to pin is that both readings are
differences of PACKET stamps rather than tick stamps, so the tests drive the
tick at times deliberately offset from the arrivals. A measurement that
picked up the tick clock would read those offsets and fail.
```

## test/test_tree_detector.cpp

### tree-thick-trunk-fixture

**The thick, noisy trunk fixture** — in `addThickTrunk`, attached to `void addThickTrunk(std::vector<SemVoxel>& out, float cx, float cy, float base_z,` (line 96)

```text
A trunk surface with THICKNESS and jitter, not the idealised single-radius
shell addTrunk() lays down. This is what a real fused voxel map holds: the
surface spans a couple of voxels radially and the returns are noisy. It is
also the case the thin-shell tests could not expose -- see
OneSidedThickTrunkIsUnderInformed below.
```

### tree-nan-pocc-entropy

**A NaN p_occ must not poison the score** — in `TreeDetector.NonFinitePOccDoesNotPoisonTheScore`, attached to `TEST(TreeDetector, NonFinitePOccDoesNotPoisonTheScore) {` (line 162)

```text
A non-finite p_occ from an upstream producer must not poison the score.
The occupancy gates use `p_occ < occ_thresh`, which is FALSE for NaN, so a
poisoned voxel is kept rather than skipped and reaches the entropy sum. If
normEntropy let it through, mean_entropy -> info_deficit would go NaN, the
`deficit > deficit_thresh` test would compare false (the tree silently reads
well-observed and is never targeted), and the std::sort comparator on
info_deficit would stop being a strict weak ordering. Both must hold: the
outputs stay finite, AND the one-sided trunk stays flagged.
```

### tree-thick-one-sided-regression

**One-sided thick trunk reads low coverage** — in `TreeDetector.OneSidedThickTrunkIsUnderInformed`, attached to `TEST(TreeDetector, OneSidedThickTrunkIsUnderInformed) {` (line 194)

```text
Two well-separated trunks segment into two distinct detections.
REGRESSION (map-test-2 bag): a trunk seen from one side only must read low
angular coverage even when its surface is thick and noisy rather than an
idealised single-radius arc.

The thin-shell OneSidedTrunkIsUnderInformed above passed throughout the
period this was broken, because with a one-voxel-thick arc even a biased
centre leaves the azimuths clustered. Give the surface real thickness and the
old per-component-median centre lands ON the arc, the voxels fan out around
it through every sector, and coverage reads ~1.0 -- a half-observed tree
scoring as fully covered. On the real map that pinned 13 of 17 trunks at
coverage 1.00 and made the emission gate arithmetically unreachable.
```

### tree-partial-arc-coverage-bias

**Why the coverage bound is 0.6** — in `TEST`, attached to `EXPECT_LT(d[0].angular_coverage, 0.6f);` (line 212)

```text
120 deg of 360 was observed, so the ideal reading is 0.33. The measured
value is 0.50: fitting a circle to a partial arc whose radial noise is a
sizeable fraction of its radius (a 0.4 m trunk on a 0.2 m grid -- the real
regime, not a pathological one) still biases the centre slightly toward the
arc, which spreads the azimuths wider than the arc truly spans. That
residual bias is why the node prefers bearing coverage, which measures
viewing geometry directly instead of inferring it from surface shape.

The value this test actually pins down is that it is no longer ~1.0. Before
the fit it read 1.00 here, i.e. "fully circled" for a trunk seen from 120
degrees, which is what made the emission gate unreachable on real maps.
```

### tree-ground-no-merge-slice

**Ground must not merge two trunks** — in `TreeDetectorGeometric.GroundDoesNotMergeTrunks`, attached to `TEST(TreeDetectorGeometric, GroundDoesNotMergeTrunks) {` (line 314)

```text
The shared ground plane must NOT merge two trunks. stem_slice_lo is zeroed
so the clustering slice would include any ground the margin gate failed to
strip: this passes only if terrain removal actually removes the plane (with
the default slice, ground below stem_slice_lo never reached the clusterer
and the test could not fail even with terrain removal ablated).
```

### tree-sloped-terrain-interp

**Terrain interpolation on a slope** — in `TreeDetectorGeometric.HandlesSlopedTerrain`, attached to `TEST(TreeDetectorGeometric, HandlesSlopedTerrain) {` (line 331)

```text
Terrain interpolation keeps ground out on a 27 deg slope (grade 0.5). With
a piecewise-constant terrain lookup, ground leaked past the margin gate at
grades above ~ground_margin/terrain_cell (~22 deg): bases dropped below the
true ground and steeper slopes fused trunks with ground ribbons.
```

### tree-sloped-base-bounds

**Tree base bounds on sloped ground** — in `TEST`, attached to `std::vector<float> zs = {d[0].center.z(), d[1].center.z()};` (line 352)

```text
Bases stay within [local ground, ground + margin]. The min-z terrain
estimate sits ~grade*cell/2 below true ground, so downslope-side trunk
voxels right at the base can survive the margin gate (base = ground
exactly) — what must never happen is a base BELOW local ground (the old
piecewise-constant lookup reported -0.5 here) or ground voxels attaching.
```

### tree-wall-verticality-sole-rejector

**Verticality alone rejects the wall** — in `TreeDetectorGeometric.RejectsWallByShape`, attached to `TEST(TreeDetectorGeometric, RejectsWallByShape) {` (line 365)

```text
A wall slab: linearity does NOT reject it (a wall longer than its in-slice
height reads linear along its length) and its median radius sits exactly AT
max_radius (1.0, not >), so the verticality gate is the sole rejector.
The second config ablates that gate (and moves the radius knife-edge out of
the way) to pin the rejection on verticality specifically.
```

## test/test_vantage_planner.cpp

### vantage-trunk-carve-out

**The trunk-surface carve-out** — in `TEST`, attached to `VantagePlanner vp(defaultCfg());` (line 136)

```text
An occupied voxel at the trunk surface (within `radius` of the axis) IS the
trunk and must not count as an occluder. The carve-out works by stopping the
march `radius + one voxel` short of the axis. To make this non-vacuous, the
occluder sits ON the sightline at x == radius — a point the march WOULD
sample if the radius term were dropped from the stop distance. So this test
fails if the carve-out regresses, not merely because the voxel is off-ray.
```

### vantage-los-off-axis-exact-traversal

**Off-axis sightline needs exact traversal** — in `VantagePlanner.LineOfSightOffAxisOccluderIsNotSkipped`, attached to `TEST(VantagePlanner, LineOfSightOffAxisOccluderIsNotSkipped) {` (line 174)

```text
The first of the off-axis LoS tests, and the bearing the four +x ones cannot
reach.

The four above it — LineOfSightClearOnEmptyMap, LineOfSightBlockedByOccluder,
TrunkSurfaceIsCarvedOut, OccluderJustBeyondTrunkSurfaceBlocks — all march
along +x, the one family of bearings where the ray enters exactly one voxel
per `res` of travel, so stepping the sampler by `res` happens to be
sufficient and those tests are bit-identical whether the spacing is correct
or not. Off-axis, a ray enters up to sqrt(3) voxels per `res` (the L1 norm of
its unit direction) and a `res`-spaced sampler simply never looks at the
surplus.

This header said "the other four" and "EVERY existing test in this file
marches along +x" until 2026-09-18, when there were exactly five LoS tests.
There are eight now and three of the additions are off-axis, immediately
below this one: LineOfSightDoesNotReadVoxelsTheRayMisses (the over-reading
half of the guarantee), LineOfSightAtFortyFiveDegreesCrossesOnlyTheDiagonal
(the exact-corner case) and LineOfSightOnANegativeBearingReadsItsFinalVoxel.
Counting the file's own tests inside one of them is a claim that goes stale
on the next TEST() appended, as this one did; what is durable is that the +x
four cannot discriminate any sampling scheme, which is why this one exists.

Geometry below, worked out at res = 0.1 on the (2,1)/sqrt(5) bearing from
voxel-centre (0.05, 0.05). A `res`-spaced march samples t = 0.1 in voxel
(1,0) and t = 0.2 in voxel (2,1) — but the ray crosses y = 0.1 at t = 0.1118
and x = 0.2 at t = 0.1677, so between those two samples it passes through
voxel (1,1) and the old sampler never looked. An occluder parked there
reported CLEAR, sending the exploitation planner to a viewpoint that cannot
see the trunk it was chosen for.

This voxel is ALSO the case that kills the obvious cheap repair. Spacing the
march by `res / L1(dir)` — the MEAN voxel-traversal length — does not fix it:
the ray occupies (1,1) for 0.0559 of travel while that spacing is 0.0745, so
the sampler lands at t = 0.1 and t = 0.1745 and straddles the window again.
No fixed spacing can work, because a ray clips a corner for arbitrarily
little travel; only an exact boundary-to-boundary traversal does. Keep this
test on this exact bearing: it discriminates between the two, which is the
whole reason it exists.
```

### vantage-los-no-over-read

**Sightline must not over-read neighbours** — in `VantagePlanner.LineOfSightDoesNotReadVoxelsTheRayMisses`, attached to `TEST(VantagePlanner, LineOfSightDoesNotReadVoxelsTheRayMisses) {` (line 240)

```text
The other half of the traversal guarantee, and the failure mode the test
above cannot see.

"Never miss a crossed voxel" is trivially satisfiable by a sampler that
reads too much — one that walks the neighbours, or samples exactly on a
boundary and picks up whichever side rounding lands on. That sampler passes
LineOfSightOffAxisOccluderIsNotSkipped and is still wrong: it reports every
vantage blocked whenever anything is parked near the sightline, which
silently empties the exploitation planner's candidate set instead of
mis-ranking it. Worse to debug, because nothing is ever chosen badly.

Same bearing, same ray, which has slope dy/dx = 1/2 from (0.05, 0.05): it
crosses y = 0.1 at x = 0.15 and x = 0.2 at y = 0.125, so the voxel run is
(1,0) -> (1,1) -> (2,1) -> ... Voxel (0,1), centre (0.05, 0.15), shares an
edge with the run's start and is not in it; voxel (2,0), centre
(0.25, 0.05), sits directly under (2,1) and is not in it either. Note that
(1,0) — centre (0.15, 0.05) — IS crossed and would be the wrong choice here.
```

### vantage-los-45-deg-diagonal

**The exact-corner 45 degree bearing** — in `VantagePlanner.LineOfSightAtFortyFiveDegreesCrossesOnlyTheDiagonal`, attached to `TEST(VantagePlanner, LineOfSightAtFortyFiveDegreesCrossesOnlyTheDiagonal) {` (line 278)

```text
The exact-corner bearing, which is the only one that reaches the traversal's
multi-axis advance.

On 45 degrees from a voxel centre, x and y cross their boundaries at the
same parameter every time: the ray goes corner to corner through (1,1),
(2,2), (3,3), ... and touches nothing else. What this pins is that the
traversal visits that diagonal and does NOT read the off-diagonal voxels
either side of each corner — the direction a sampler errs in if it walks
neighbours or rounds a corner sample the wrong way.

What it does NOT pin, checked by mutation rather than assumed: replacing the
all-tied-axes advance with the textbook single-axis `break` form still
passes. That mutant is equivalent here, not merely uncaught — the axis left
un-advanced yields a zero-width interval on the next iteration whose
midpoint IS the corner, and the corner floors into the diagonal voxel the
ray is entering. It costs a duplicate sample per corner and reads nothing
new. The loop advances all tied axes anyway because that is the form whose
invariant ("t_max[a] is always the NEXT crossing ahead of t_enter") can be
stated, not because a test here would catch the alternative.
```

### vantage-los-negative-bearing-midpoint

**Negative bearing needs midpoint sampling** — in `VantagePlanner.LineOfSightOnANegativeBearingReadsItsFinalVoxel`, attached to `TEST(VantagePlanner, LineOfSightOnANegativeBearingReadsItsFinalVoxel) {` (line 324)

```text
A ray running in the NEGATIVE direction, which no other test in this file
uses, and the case that makes the midpoint sample load-bearing.

The traversal could sample each interval at its entry instead of its middle,
and on every other test here that is indistinguishable: the entry is a voxel
boundary, and floor() keying resolves a boundary to the voxel on its far
side — which, going positive, is the voxel being ENTERED. Going negative it
is the voxel being LEFT, so entry-sampling reads the run shifted one voxel
late and simply never looks at the last voxel before `stop`. A midpoint is
strictly interior and has no such dependence on which way rounding falls.

Geometry at res = 0.1, bearing (-2,-1)/sqrt(5) from (0.95, 0.95). The
carve-out puts the first sample at t = 0.1 inside voxel (8,9); the ray
crosses y = 0.9 at t = 0.1118 and enters (8,8). `stop` is set to 0.14 —
short of the next crossing — so (8,8) is the FINAL voxel, with no later
boundary at which a late reader could pick it up by accident.
```
