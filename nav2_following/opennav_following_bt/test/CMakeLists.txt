# Following test
ament_add_gtest(test_follow_object_action test_follow_object_action.cpp)
target_link_libraries(test_follow_object_action
  behaviortree_cpp::behaviortree_cpp
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  opennav_follow_action_bt_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

# Following cancel test
ament_add_gtest(test_follow_object_cancel_node test_follow_object_cancel_node.cpp)
target_link_libraries(test_follow_object_cancel_node
  behaviortree_cpp::behaviortree_cpp
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  opennav_follow_cancel_bt_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)
