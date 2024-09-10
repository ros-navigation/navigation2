add_executable(dummy_controller_node
  src/dummy_controller/main.cpp
  src/dummy_controller/dummy_controller.cpp
)
target_link_libraries(dummy_controller_node PRIVATE
  ${geometry_msgs_TARGETS}
  nav2_behavior_tree::nav2_behavior_tree
  nav2_util::nav2_util_core
  rclcpp::rclcpp
)
