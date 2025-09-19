# Tests
ament_add_gtest(velocity_smoother_tests
  test_velocity_smoother.cpp
)
target_link_libraries(velocity_smoother_tests
  ${library_name}
  ${geometry_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
