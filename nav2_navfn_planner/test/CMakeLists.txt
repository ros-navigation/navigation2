# Test dynamic parameters
nav2_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test NavFn path extraction cycle limit
nav2_add_gtest(test_max_cycles
  test_max_cycles.cpp
)
target_link_libraries(test_max_cycles
  ${library_name}
)
