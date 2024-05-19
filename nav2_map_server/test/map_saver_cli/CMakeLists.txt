# map_saver CLI
ament_add_gtest(test_map_saver_cli
  test_map_saver_cli.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)
target_link_libraries(test_map_saver_cli
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  stdc++fs
)
