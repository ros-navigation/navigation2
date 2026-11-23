nav2_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
)

nav2_add_gtest(test_pose_persistence
  test_pose_persistence.cpp
)
target_link_libraries(test_pose_persistence
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
)

nav2_add_gtest(test_map_conversion
  test_map_conversion.cpp
)
target_link_libraries(test_map_conversion
  ${library_name}
  rclcpp::rclcpp
)
