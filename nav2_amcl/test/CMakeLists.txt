nav2_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
)

nav2_add_gtest(test_random_seed_parameter
  test_random_seed_parameter.cpp
)
target_link_libraries(test_random_seed_parameter
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
)
