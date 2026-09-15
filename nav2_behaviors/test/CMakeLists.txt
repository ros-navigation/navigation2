nav2_add_gtest(test_behaviors
  test_behaviors.cpp
)
target_link_libraries(test_behaviors rclcpp::rclcpp rclcpp_action::rclcpp_action nav2_msgs::nav2_msgs ${library_name})

nav2_add_gtest(test_behavior_pose test_behavior_pose.cpp)
target_link_libraries(test_behavior_pose
  nav2_spin_behavior
  nav2_drive_on_heading_behavior
  nav2_back_up_behavior
  nav2_assisted_teleop_behavior
)
