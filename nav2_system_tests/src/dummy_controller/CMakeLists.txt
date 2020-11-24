add_executable(dummy_controller_node
  src/dummy_controller/main.cpp
  src/dummy_controller/dummy_controller.cpp
)

ament_target_dependencies(dummy_controller_node
  rclcpp
  std_msgs
  nav2_util
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
)