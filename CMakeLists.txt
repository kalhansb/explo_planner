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
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ==================================================================
# Library
# ==================================================================

add_library(explo_planner_lib
  src/scoring.cpp
  src/map_cache.cpp
  src/candidate_generator.cpp
  src/fov_evaluator.cpp
  src/metrics_logger.cpp
  src/cost_grid.cpp
  src/coordination.cpp
  src/plan_map_query.cpp
  src/failed_goal_blacklist.cpp
  src/planner_util.cpp
  src/target_queue.cpp
  src/vantage_planner.cpp
)

target_include_directories(explo_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(explo_planner_lib
  PUBLIC Eigen3::Eigen
)

ament_target_dependencies(explo_planner_lib PUBLIC
  rclcpp sensor_msgs nav_msgs scovox_core scovox_msgs
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

ament_target_dependencies(explo_planner_node
  rclcpp geometry_msgs nav_msgs sensor_msgs
  visualization_msgs scovox_core scovox_msgs tf2 tf2_ros tf2_geometry_msgs
)

# Time-based tree-target publisher: drives the exploitation targets topic from
# a preselected, time-scheduled list (stand-in for a future tree detector).
add_executable(target_scheduler_node
  src/target_scheduler_node.cpp
)

ament_target_dependencies(target_scheduler_node
  rclcpp scovox_msgs
)

# ==================================================================
# Install
# ==================================================================

install(TARGETS explo_planner_node target_scheduler_node
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

  ament_add_gtest(test_fov_evaluator test/test_fov_evaluator.cpp)
  target_link_libraries(test_fov_evaluator explo_planner_lib)

  ament_add_gtest(test_cost_grid test/test_cost_grid.cpp)
  target_link_libraries(test_cost_grid explo_planner_lib)

  ament_add_gtest(test_coordination test/test_coordination.cpp)
  target_link_libraries(test_coordination explo_planner_lib)

  ament_add_gtest(test_plan_map_query test/test_plan_map_query.cpp)
  target_link_libraries(test_plan_map_query explo_planner_lib)

  ament_add_gtest(test_failed_goal_blacklist test/test_failed_goal_blacklist.cpp)
  target_link_libraries(test_failed_goal_blacklist explo_planner_lib)

  ament_add_gtest(test_planner_util test/test_planner_util.cpp)
  target_link_libraries(test_planner_util explo_planner_lib)

  ament_add_gtest(test_target_queue test/test_target_queue.cpp)
  target_link_libraries(test_target_queue explo_planner_lib)

  ament_add_gtest(test_vantage_planner test/test_vantage_planner.cpp)
  target_link_libraries(test_vantage_planner explo_planner_lib)
endif()

ament_package()
