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

# Standalone EIG planner: self-contained node + main() in one file. This is
# the base the exploration/exploitation system is built on, hard-wired to the
# SCovox Beta EIG scorer (no planner_type knob).
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

# Comparison node: same planner core but exposes all planner_type options
# (eig | entropy | frontier | random | ssmi) for the experiments. The node
# class (state machine + ROS plumbing) lives in exploration_planner_node.cpp;
# exploration_planner_main.cpp is just main().
add_executable(exploration_planner_node
  src/exploration_planner_main.cpp
  src/exploration_planner_node.cpp
)

target_link_libraries(exploration_planner_node
  explo_planner_lib
)

ament_target_dependencies(exploration_planner_node
  rclcpp geometry_msgs nav_msgs sensor_msgs
  visualization_msgs scovox_core scovox_msgs tf2 tf2_ros tf2_geometry_msgs
)

# ==================================================================
# Install
# ==================================================================

install(TARGETS explo_planner_node exploration_planner_node
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
endif()

ament_package()
