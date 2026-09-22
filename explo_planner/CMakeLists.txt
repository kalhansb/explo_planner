cmake_minimum_required(VERSION 3.16)
project(explo_planner)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(scovox_core REQUIRED)
find_package(scovox_msgs REQUIRED)
find_package(explo_planner_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ==================================================================
# Provenance
# ==================================================================
# Stamp the source revision into the planner binary so every experiment event
# log can say which code produced it. No derived artefact in this project
# carried provenance before, and the metrics CSV's schema changed silently
# between campaigns with nothing in the data to date it.
#
# Stamped at BUILD time, not configure time, into a generated header. This used
# to be an execute_process() right here, which runs exactly once — when the
# build tree is configured — so every later source edit rebuilt the binary
# while leaving the old revision compiled into it. Campaign cr5 ran a binary
# stamped 6ce7ad0 against a manifest recording 1a097dc for that reason, and the
# gate's check 3b, whose whole job is comparing those two, could not pass on a
# single cell of it.
#
# The custom target re-runs the query on every build; the script writes the
# header only when the answer changes, so the extra recompile happens on the
# builds after a commit and not otherwise. Degrades to "unknown" outside a git
# checkout rather than failing the build.
set(EXPLO_PLANNER_GIT_REV_HDR
    "${CMAKE_CURRENT_BINARY_DIR}/generated/explo_planner_git_rev.h")
add_custom_target(explo_planner_git_rev_hdr ALL
  BYPRODUCTS "${EXPLO_PLANNER_GIT_REV_HDR}"
  COMMAND ${CMAKE_COMMAND}
          -DOUT=${EXPLO_PLANNER_GIT_REV_HDR}
          -DSRC=${CMAKE_CURRENT_SOURCE_DIR}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/StampGitRev.cmake
  COMMENT "Stamping explo_planner git revision")
# Configure time too, so the header EXISTS before the first compile in a clean
# build tree. The target above overwrites it with the same answer moments later.
execute_process(
  COMMAND ${CMAKE_COMMAND}
          -DOUT=${EXPLO_PLANNER_GIT_REV_HDR}
          -DSRC=${CMAKE_CURRENT_SOURCE_DIR}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/StampGitRev.cmake)

# ==================================================================
# Library
# ==================================================================

add_library(explo_planner_lib
  src/scoring.cpp
  src/map_cache.cpp
  src/candidate_generator.cpp
  src/fov_evaluator.cpp
  src/metrics_logger.cpp
  src/experiment_log.cpp
  src/cost_grid.cpp
  src/coordination.cpp
  src/proximity_guard.cpp
  src/plan_map_query.cpp
  src/failed_goal_blacklist.cpp
  src/planner_util.cpp
  src/fleet_identity.cpp
  src/cell_world.cpp
  src/team_model.cpp
  src/exchange_drain.cpp
  src/global_allocator.cpp
  src/reconnect_gate.cpp
  src/rendezvous_scheduler.cpp
  src/pursuit_predictor.cpp
  src/target_queue.cpp
  src/vantage_planner.cpp
  src/tree_detector.cpp
  src/separation.cpp
)

target_include_directories(explo_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(explo_planner_lib
  PUBLIC Eigen3::Eigen
)

ament_target_dependencies(explo_planner_lib PUBLIC
  rclcpp sensor_msgs nav_msgs scovox_core scovox_msgs explo_planner_msgs
)

# ==================================================================
# Node executables
# ==================================================================

# EIG NBV planner: self-contained node + main() in one file. Hard-wired to the
# SCovox Beta expected-information-gain scorer (no planner_type knob). Multi-
# robot MinPos coordination is built in and param-gated (coordination_enabled).
add_executable(explo_planner_node
  src/explo_planner_node.cpp
)

target_link_libraries(explo_planner_node
  explo_planner_lib
)

# Provenance for the experiment event log's run_start event (see above). The
# revision arrives through the generated header rather than a compile
# definition, because a definition is fixed at configure time and that is the
# staleness this replaced.
add_dependencies(explo_planner_node explo_planner_git_rev_hdr)
target_include_directories(explo_planner_node PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

ament_target_dependencies(explo_planner_node
  rclcpp geometry_msgs nav_msgs sensor_msgs
  visualization_msgs scovox_core scovox_msgs explo_planner_msgs
  tf2 tf2_ros tf2_geometry_msgs rclcpp_action nav2_msgs std_msgs
)

# Time-based tree-target publisher: drives the exploitation targets topic from
# a preselected, time-scheduled list (stand-in for a future tree detector).
add_executable(target_scheduler_node
  src/target_scheduler_node.cpp
)

ament_target_dependencies(target_scheduler_node
  rclcpp explo_planner_msgs
)

# Live tree detector: segments trees out of the fused dscovox map and publishes
# the under-informed ones as TreeTarget — the real detector target_scheduler_node
# stands in for. Drop-in on the same targets topic.
add_executable(tree_detector_node
  src/tree_detector_node.cpp
)

target_link_libraries(tree_detector_node
  explo_planner_lib
)

ament_target_dependencies(tree_detector_node
  rclcpp scovox_msgs explo_planner_msgs tf2 tf2_ros geometry_msgs
)

# ==================================================================
# Install
# ==================================================================

install(TARGETS
  explo_planner_node target_scheduler_node tree_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ==================================================================
# Tests
# ==================================================================

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_scoring test/test_scoring.cpp)
  target_link_libraries(test_scoring explo_planner_lib)

  ament_add_gtest(test_candidate_generator test/test_candidate_generator.cpp)
  target_link_libraries(test_candidate_generator explo_planner_lib)

  ament_add_gtest(test_map_cache test/test_map_cache.cpp)
  target_link_libraries(test_map_cache explo_planner_lib)

  ament_add_gtest(test_fov_evaluator test/test_fov_evaluator.cpp)
  target_link_libraries(test_fov_evaluator explo_planner_lib)

  ament_add_gtest(test_cost_grid test/test_cost_grid.cpp)
  target_link_libraries(test_cost_grid explo_planner_lib)

  ament_add_gtest(test_coordination test/test_coordination.cpp)
  target_link_libraries(test_coordination explo_planner_lib)

  ament_add_gtest(test_proximity_guard test/test_proximity_guard.cpp)
  target_link_libraries(test_proximity_guard explo_planner_lib)

  ament_add_gtest(test_plan_map_query test/test_plan_map_query.cpp)
  target_link_libraries(test_plan_map_query explo_planner_lib)

  ament_add_gtest(test_failed_goal_blacklist test/test_failed_goal_blacklist.cpp)
  target_link_libraries(test_failed_goal_blacklist explo_planner_lib)
  # Point the test at the SOURCE config, not the installed copy: the claim it
  # guards is about the value that gets committed and shipped, and the install
  # tree can be stale or absent when the tests run.
  target_compile_definitions(test_failed_goal_blacklist PRIVATE
    SHARED_PARAMS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/shared_params.yaml")

  ament_add_gtest(test_metrics_logger test/test_metrics_logger.cpp)
  target_link_libraries(test_metrics_logger explo_planner_lib)

  ament_add_gtest(test_home_trail test/test_home_trail.cpp)
  target_link_libraries(test_home_trail explo_planner_lib)

  ament_add_gtest(test_experiment_log test/test_experiment_log.cpp)
  target_link_libraries(test_experiment_log explo_planner_lib)
  # The declared event vocabulary (kEventKinds) is checked against the WRITER's
  # own source, not against a copy of the list — same reasoning as the shared
  # params path below. Source tree, not install tree: the claim is about the
  # code that gets committed, and the installed copy can be stale or absent.
  target_compile_definitions(test_experiment_log PRIVATE
    EXPERIMENT_LOG_CPP="${CMAKE_CURRENT_SOURCE_DIR}/src/experiment_log.cpp")

  ament_add_gtest(test_planner_util test/test_planner_util.cpp)
  target_link_libraries(test_planner_util explo_planner_lib)

  ament_add_gtest(test_fleet_identity test/test_fleet_identity.cpp)
  target_link_libraries(test_fleet_identity explo_planner_lib)

  ament_add_gtest(test_cell_world test/test_cell_world.cpp)
  target_link_libraries(test_cell_world explo_planner_lib)

  ament_add_gtest(test_team_model test/test_team_model.cpp)
  target_link_libraries(test_team_model explo_planner_lib)

  # Gen-33 drain release, run rather than scanned: test-plan 7 and the
  # behavioural half of 8. The predicate was extracted from doReturnSync into
  # the lib for this; the node's wiring of it stays a scan in
  # test_gen21_latched_hold.
  ament_add_gtest(test_exchange_drain test/test_exchange_drain.cpp)
  target_link_libraries(test_exchange_drain explo_planner_lib)

  ament_add_gtest(test_global_allocator test/test_global_allocator.cpp)
  target_link_libraries(test_global_allocator explo_planner_lib)

  ament_add_gtest(test_reconnect_gate test/test_reconnect_gate.cpp)
  target_link_libraries(test_reconnect_gate explo_planner_lib)

  ament_add_gtest(test_rendezvous_scheduler test/test_rendezvous_scheduler.cpp)
  target_link_libraries(test_rendezvous_scheduler explo_planner_lib)

  # T5. The endpoint contract: the three events that decide when a run is over.
  #
  # It said "plus the departure arithmetic's edges" until 2026-09-18, and those
  # nine tests were deleted with RendezvousScheduler::shouldDepart in generation
  # 19 — departure is no longer a `remaining <= need` comparison, so there are
  # no edges left to pin. See the GROUP A tombstone in test_endpoint.cpp, which
  # records what they asserted and which one hazard outlived them.
  #
  # What remains is the logged rows (GROUP B) and two source scans of the node
  # (GROUPS C and E). The split from test_experiment_log stands on the first of
  # those alone: that file owns the writer, this one owns the ORDER and the
  # once-per-run-ness of what the writer emits, which is the property every
  # campaign's primary metric is a stamp on.
  ament_add_gtest(test_endpoint test/test_endpoint.cpp)
  target_link_libraries(test_endpoint explo_planner_lib)
  # The once-per-run mission-return guard lives in explo_planner_node.cpp, which
  # is NOT in explo_planner_lib's source list above, so nothing links it and no
  # test can call it. The guard is still the load-bearing half of F1, so
  # test_endpoint reads the node's SOURCE and asserts the latch is present and
  # ordered — the same technique, and the same justification, as
  # test_experiment_log reading the writer for kEventKinds. Source tree, not
  # install tree: the claim is about the code being built.
  target_compile_definitions(test_endpoint PRIVATE
    EXPLO_PLANNER_NODE_CPP="${CMAKE_CURRENT_SOURCE_DIR}/src/explo_planner_node.cpp")

  ament_add_gtest(test_pursuit_predictor test/test_pursuit_predictor.cpp)
  target_link_libraries(test_pursuit_predictor explo_planner_lib)

  ament_add_gtest(test_target_queue test/test_target_queue.cpp)
  target_link_libraries(test_target_queue explo_planner_lib)

  ament_add_gtest(test_vantage_planner test/test_vantage_planner.cpp)
  target_link_libraries(test_vantage_planner explo_planner_lib)

  ament_add_gtest(test_tree_detector test/test_tree_detector.cpp)
  target_link_libraries(test_tree_detector explo_planner_lib)

  ament_add_gtest(test_separation test/test_separation.cpp)
  target_link_libraries(test_separation explo_planner_lib)

  # T9. Generation 20's rendezvous invariants. Every one of them lives in the
  # node — the countdown's write-before-arm ordering, the five-site separation
  # predicate, the appointment-manoeuvre latch's keying — so this is the same
  # source-scan technique as test_endpoint's GROUP C, for the same reason and
  # with the same stated limits. It links the library only for gtest's sake; it
  # calls nothing from it.
  ament_add_gtest(test_gen20_rendezvous test/test_gen20_rendezvous.cpp)
  target_link_libraries(test_gen20_rendezvous explo_planner_lib)
  target_compile_definitions(test_gen20_rendezvous PRIVATE
    EXPLO_PLANNER_NODE_CPP="${CMAKE_CURRENT_SOURCE_DIR}/src/explo_planner_node.cpp")

  # T10. Generation 21's at-the-rendezvous hold. Same source-scan technique as
  # T9 and for the same reason (the node is not in the library's source list),
  # but a separate target on purpose: three of four invariants are guards on
  # paths in functions a reader of the hold never opens, and each is individually
  # sufficient to reintroduce an asymmetric corruption of the primary endpoint.
  ament_add_gtest(test_gen21_latched_hold test/test_gen21_latched_hold.cpp)
  target_link_libraries(test_gen21_latched_hold explo_planner_lib)
  target_compile_definitions(test_gen21_latched_hold PRIVATE
    EXPLO_PLANNER_NODE_CPP="${CMAKE_CURRENT_SOURCE_DIR}/src/explo_planner_node.cpp")

  # T11. Generation 22's pre-mission hold. Same source-scan technique as T9 and
  # T10, and a separate target for the same reason: the hold is four edits in
  # four places plus one invariant that is not an edit at all (heartbeatTick's
  # handshake must keep running while a robot is held, or the hold is a 60 s
  # cost in all four arms that buys nothing).
  #
  # THIS IS THE FIRST TEST THAT SCANS THE HARNESS, and the second compile
  # definition is why it has to. mission_start_hold_sec defaults ON in the node,
  # so "the harness forgot it" is not the risk; the risk is the harness passing
  # it in the ARM-CONDITIONAL `EXTRA` array rather than the unconditional
  # per-robot argv, which would apply the hold to rendezvous and hybrid only and
  # turn a startup cost into an apparent treatment effect in exactly the two
  # arms under test. That is a property of run_explo_sim_rviz.sh and of nothing
  # in the C++, so the C++ cannot be where it is checked.
  ament_add_gtest(test_gen22_start_hold test/test_gen22_start_hold.cpp)
  target_link_libraries(test_gen22_start_hold explo_planner_lib)
  target_compile_definitions(test_gen22_start_hold PRIVATE
    EXPLO_PLANNER_NODE_CPP="${CMAKE_CURRENT_SOURCE_DIR}/src/explo_planner_node.cpp"
    EXPLO_PLANNER_SIM_SH="${CMAKE_CURRENT_SOURCE_DIR}/../sim/run_explo_sim_rviz.sh")

  # T12. Generation 23's contagious arming: one robot's broken view of the team
  # arms the whole team, in one hop, over a new TeamWorld field.
  #
  # THE FIRST OF THESE THAT IS HALF EXECUTABLE, and the split is deliberate.
  # The wire field's storage semantics live in TeamModel, which IS in
  # explo_planner_lib's source list, so "the bit attaches to its sender",
  # "gossip never relays it" and "it has no TTL of its own" are CALLED rather
  # than scanned — and those are the three whose failure is silent, because a
  # relayed claim still produces a plausible-looking armed team. Everything
  # downstream (the five agreement sites, the producer's own-read-not-armed-
  # state rule, the finished-peer exemption) is in explo_planner_node.cpp,
  # which is not in that list, so it carries the same source-scan limit as
  # T9-T11. There is no glob in this file to blame for that — the list above is
  # explicit — but do not read the limit as one line away from lifting: the
  # node TU also defines main(), so a test that pulls any node symbol out of
  # the archive collides with gtest_main's main(). Making the node testable
  # means splitting main() off first, not editing this list.
  #
  # A separate target from T9-T11 for the usual reason and one more: generations
  # 18 and 19 were BOTH the five-site rule broken, in opposite directions, and
  # both reached a smoke run green. This target is the standing check that arm
  # and release read one predicate, and it must stay runnable against a mutated
  # node on its own.
  ament_add_gtest(test_gen23_contagion test/test_gen23_contagion.cpp)
  target_link_libraries(test_gen23_contagion explo_planner_lib)
  target_compile_definitions(test_gen23_contagion PRIVATE
    EXPLO_PLANNER_NODE_CPP="${CMAKE_CURRENT_SOURCE_DIR}/src/explo_planner_node.cpp")
endif()

ament_package()
