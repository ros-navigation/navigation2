# Test dynamic parameters
nav2_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
)

nav2_add_gtest(test_plan_through_poses
  test_plan_through_poses.cpp
)
target_link_libraries(test_plan_through_poses
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  nav2_costmap_2d::nav2_costmap_2d_core
  ${rcl_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
)
