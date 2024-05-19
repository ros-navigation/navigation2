# map_io unit test
ament_add_gtest(test_map_io test_map_io.cpp ${PROJECT_SOURCE_DIR}/test/test_constants.cpp)
target_link_libraries(test_map_io rclcpp::rclcpp ${nav_msgs_TARGETS} ${map_io_library_name} stdc++fs)
target_include_directories(test_map_io
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/test>")

# costmap_filter_info_server unit test
ament_add_gtest(test_costmap_filter_info_server
  test_costmap_filter_info_server.cpp
)
target_link_libraries(test_costmap_filter_info_server rclcpp::rclcpp ${library_name} ${map_io_library_name})
