# Test dynamic parameters
nav2_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rcl_interfaces::rcl_interfaces
)

nav2_add_gtest(test_plan_through_poses
  test_plan_through_poses.cpp
)
target_link_libraries(test_plan_through_poses
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  nav2_costmap_2d::nav2_costmap_2d_core
  rcl_interfaces::rcl_interfaces
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  nav2_msgs::nav2_msgs
)
