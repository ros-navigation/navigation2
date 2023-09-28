include_directories(${PROJECT_SOURCE_DIR}/test)

# map_io unit test
ament_add_gtest(test_map_io test_map_io.cpp ${PROJECT_SOURCE_DIR}/test/test_constants.cpp)

ament_target_dependencies(test_map_io rclcpp nav_msgs)

target_link_libraries(test_map_io
  ${map_io_library_name}
)

# costmap_filter_info_server unit test
ament_add_gtest(test_costmap_filter_info_server
  test_costmap_filter_info_server.cpp
)

ament_target_dependencies(test_costmap_filter_info_server rclcpp)

target_link_libraries(test_costmap_filter_info_server
  ${library_name}
)
