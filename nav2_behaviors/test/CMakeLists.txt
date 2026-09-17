nav2_add_gtest(test_behaviors
  test_behaviors.cpp
)
target_link_libraries(test_behaviors rclcpp::rclcpp rclcpp_action::rclcpp_action nav2_msgs::nav2_msgs ${library_name})

nav2_add_gtest(test_assisted_teleop
  test_assisted_teleop.cpp
)
target_link_libraries(test_assisted_teleop nav2_assisted_teleop_behavior)
