# Test dynamic parameters
ament_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
)
