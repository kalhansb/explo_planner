# Moved comments: doc/explo_planner_msgs_code_notes.md
cmake_minimum_required(VERSION 3.16)
project(explo_planner_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TreeTarget.msg"
  "msg/RobotIntent.msg"
  # The shared coarse world model, as new types rather than new RobotIntent
  # fields: on Humble an added RobotIntent field silently hides a mixed-build
  # producer from an updated consumer. See TeamWorld.msg.
  # (notes: msgs-new-types-not-fields)
  "msg/CellState.msg"
  "msg/TeamWorld.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
