ament_add_gtest(test_behavior_tree_node
  test_behavior_tree_node.cpp
  server_handler.cpp
)
ament_target_dependencies(test_behavior_tree_node
  ament_index_cpp
  behaviortree_cpp
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  nav2_util
  rclcpp
  tf2_ros
)
# target_link_libraries(test_behavior_tree_node
#   ament_index_cpp::ament_index_cpp
#   behaviortree_cpp::behaviortree_cpp
#   ${geometry_msgs_TARGETS}
#   nav2_behavior_tree::nav2_behavior_tree
#   ${nav2_msgs_TARGETS}
#   nav2_util::nav2_util_core
#   rclcpp::rclcpp
#   tf2_ros::tf2_ros
# )
