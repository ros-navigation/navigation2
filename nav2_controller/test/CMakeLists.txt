nav2_add_gtest(test_controller_server
  test_controller_server.cpp
)
target_link_libraries(test_controller_server
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
)
