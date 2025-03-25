add_executable(dummy_planner_node
  src/dummy_planner/main.cpp
  src/dummy_planner/dummy_planner.cpp
)
ament_target_dependencies(dummy_planner_node
  nav2_behavior_tree
  rclcpp
)
