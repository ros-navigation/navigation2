ament_add_gtest(test_behaviors
  test_behaviors.cpp
)
target_link_libraries(test_behaviors rclcpp::rclcpp rclcpp_action::rclcpp_action ${nav2_msgs_TARGETS} ${library_name})
