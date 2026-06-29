// Comparison exploration planner executable.
//
// Thin entry point over ExplorationPlannerNode (exploration_planner_node.hpp /
// .cpp), the multi-planner node that exposes every planner_type option
// (eig | entropy | frontier | random | ssmi). Used for the comparison
// experiments. The EIG-only standalone planner lives in explo_planner_node.cpp.

#include "explo_planner/exploration_planner_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Single-threaded executor: the planner ingests the fused map over a topic
  // subscription (non-blocking), so there is no blocking service future to
  // service on a second thread. All callbacks and timers run on one thread,
  // which also makes the map-callback / state-machine interaction race-free.
  rclcpp::spin(std::make_shared<explo_planner::ExplorationPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
