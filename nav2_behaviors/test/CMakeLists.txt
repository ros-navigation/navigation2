nav2_add_gtest(test_behaviors
  test_behaviors.cpp
)
target_link_libraries(test_behaviors rclcpp::rclcpp rclcpp_action::rclcpp_action nav2_msgs::nav2_msgs ${library_name})
