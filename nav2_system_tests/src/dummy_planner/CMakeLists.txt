add_executable(dummy_planner_node
  src/dummy_planner/main.cpp
  src/dummy_planner/dummy_planner.cpp
)
target_link_libraries(dummy_planner_node PRIVATE
  nav2_behavior_tree::nav2_behavior_tree
  rclcpp::rclcpp
)
