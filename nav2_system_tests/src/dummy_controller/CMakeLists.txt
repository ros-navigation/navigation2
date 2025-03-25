add_executable(dummy_controller_node
  src/dummy_controller/main.cpp
  src/dummy_controller/dummy_controller.cpp
)
ament_target_dependencies(dummy_controller_node
  geometry_msgs
  nav2_behavior_tree
  nav2_util
  rclcpp
)
# target_link_libraries(dummy_controller_node PRIVATE
#   ${geometry_msgs_TARGETS}
#   nav2_behavior_tree::nav2_behavior_tree
#   nav2_util::nav2_util_core
#   rclcpp::rclcpp
# )
