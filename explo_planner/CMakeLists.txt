# Moved comments: doc/explo_planner_code_notes.md
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
# Stamps the git revision into a generated header on every build, not at
# configure time, so the event log names the code that ran. The header is
# rewritten only when the revision changes; outside git it reads "unknown".
# (notes: cmake-git-rev-stamp)
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
  src/meeting_attendance.cpp
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

  # Runs the drain-release predicate extracted from doReturnSync into the lib;
  # the node's wiring of it stays a scan in test_gen21_latched_hold.
  # (notes: cmake-test-exchange-drain)
  ament_add_gtest(test_exchange_drain test/test_exchange_drain.cpp)
  target_link_libraries(test_exchange_drain explo_planner_lib)

  # Gen-33 §10 item 5: a finished robot still comes to the meeting and says
  # DONE only once it leaves; its partner waits for it until then. The two
  # predicates run here; the node's wiring of them is a scan in
  # test_gen21_latched_hold.
  ament_add_gtest(test_meeting_attendance test/test_meeting_attendance.cpp)
  target_link_libraries(test_meeting_attendance explo_planner_lib)

  ament_add_gtest(test_global_allocator test/test_global_allocator.cpp)
  target_link_libraries(test_global_allocator explo_planner_lib)

  ament_add_gtest(test_reconnect_gate test/test_reconnect_gate.cpp)
  target_link_libraries(test_reconnect_gate explo_planner_lib)

  ament_add_gtest(test_rendezvous_scheduler test/test_rendezvous_scheduler.cpp)
  target_link_libraries(test_rendezvous_scheduler explo_planner_lib)

  # The endpoint contract: the logged rows plus two source scans of the node for
  # the three events that end a run. test_experiment_log owns the writer; this
  # owns the order and once-per-run-ness of what it emits.
  # (notes: cmake-test-endpoint-contract)
  ament_add_gtest(test_endpoint test/test_endpoint.cpp)
  target_link_libraries(test_endpoint explo_planner_lib)
  # explo_planner_node.cpp is not in explo_planner_lib, so no test can call the
  # once-per-run mission-return guard; test_endpoint reads the node's source
  # tree, not the install tree, for the latch and its order.
  # (notes: cmake-test-endpoint-node-scan)
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

  # Source-scans the node for the rendezvous invariants: write-before-arm
  # ordering, the five-site separation predicate, the appointment-manoeuvre
  # latch keying. Links the library only for gtest; calls nothing in it.
  # (notes: cmake-test-gen20-rendezvous)
  ament_add_gtest(test_gen20_rendezvous test/test_gen20_rendezvous.cpp)
  target_link_libraries(test_gen20_rendezvous explo_planner_lib)
  target_compile_definitions(test_gen20_rendezvous PRIVATE
    EXPLO_PLANNER_NODE_CPP="${CMAKE_CURRENT_SOURCE_DIR}/src/explo_planner_node.cpp")

  # Source-scans the node for the at-the-rendezvous hold. A separate target on
  # purpose: most of its invariants are guards in functions a reader of the hold
  # never opens. (notes: cmake-test-gen21-latched-hold)
  ament_add_gtest(test_gen21_latched_hold test/test_gen21_latched_hold.cpp)
  target_link_libraries(test_gen21_latched_hold explo_planner_lib)
  target_compile_definitions(test_gen21_latched_hold PRIVATE
    EXPLO_PLANNER_NODE_CPP="${CMAKE_CURRENT_SOURCE_DIR}/src/explo_planner_node.cpp")

  # Scans the node and run_explo_sim_rviz.sh: heartbeatTick's handshake must
  # keep running while a robot is held, and the harness must pass
  # mission_start_hold_sec to every robot, not via the arm-conditional EXTRA
  # array. (notes: cmake-test-gen22-start-hold)
  ament_add_gtest(test_gen22_start_hold test/test_gen22_start_hold.cpp)
  target_link_libraries(test_gen22_start_hold explo_planner_lib)
  target_compile_definitions(test_gen22_start_hold PRIVATE
    EXPLO_PLANNER_NODE_CPP="${CMAKE_CURRENT_SOURCE_DIR}/src/explo_planner_node.cpp"
    EXPLO_PLANNER_SIM_SH="${CMAKE_CURRENT_SOURCE_DIR}/../sim/run_explo_sim_rviz.sh")

  # Calls TeamModel for the armed bit's storage semantics and source-scans the
  # node for the rest. The node TU defines main(), which collides with
  # gtest_main, so making it testable means splitting main() off first.
  # (notes: cmake-test-gen23-contagion)
  ament_add_gtest(test_gen23_contagion test/test_gen23_contagion.cpp)
  target_link_libraries(test_gen23_contagion explo_planner_lib)
  target_compile_definitions(test_gen23_contagion PRIVATE
    EXPLO_PLANNER_NODE_CPP="${CMAKE_CURRENT_SOURCE_DIR}/src/explo_planner_node.cpp")
endif()

ament_package()
