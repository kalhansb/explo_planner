cmake_minimum_required(VERSION 3.16)
project(explo_planner_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TreeTarget.msg"
  "msg/RobotIntent.msg"
  # The shared coarse world model. New TYPES rather than new fields on
  # RobotIntent, deliberately: see the compatibility note at the top of
  # TeamWorld.msg — on Humble a field added to RobotIntent makes a mixed-build
  # producer permanently invisible to an updated consumer, silently, for
  # deconfliction traffic as well. An unmatched type fails loudly instead.
  "msg/CellState.msg"
  "msg/TeamWorld.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
