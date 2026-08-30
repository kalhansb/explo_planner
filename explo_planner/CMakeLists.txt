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
# Configure-time, so it identifies the checkout the build tree was CONFIGURED
# from: editing a source file and rebuilding without re-running cmake leaves it
# stale. That is why the event log also records the translation unit's compile
# timestamp, which is always exactly right. Degrades to "unknown" outside a git
# checkout (release tarball, vendored copy) rather than failing the build.
set(EXPLO_PLANNER_GIT_REV "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --always --dirty --tags
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE _explo_git_rev
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _explo_git_rc)
  if(_explo_git_rc EQUAL 0 AND NOT _explo_git_rev STREQUAL "")
    set(EXPLO_PLANNER_GIT_REV "${_explo_git_rev}")
  endif()
endif()
message(STATUS "explo_planner git revision: ${EXPLO_PLANNER_GIT_REV}")

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
  src/target_queue.cpp
  src/vantage_planner.cpp
  src/tree_detector.cpp
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

# Provenance for the experiment event log's run_start event (see above).
target_compile_definitions(explo_planner_node PRIVATE
  EXPLO_PLANNER_GIT_REV="${EXPLO_PLANNER_GIT_REV}"
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

  ament_add_gtest(test_target_queue test/test_target_queue.cpp)
  target_link_libraries(test_target_queue explo_planner_lib)

  ament_add_gtest(test_vantage_planner test/test_vantage_planner.cpp)
  target_link_libraries(test_vantage_planner explo_planner_lib)

  ament_add_gtest(test_tree_detector test/test_tree_detector.cpp)
  target_link_libraries(test_tree_detector explo_planner_lib)
endif()

ament_package()
