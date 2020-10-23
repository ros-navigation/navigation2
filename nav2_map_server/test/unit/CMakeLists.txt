include_directories(${PROJECT_SOURCE_DIR}/test)

# map_io unit test
ament_add_gtest(test_map_io_2d test_map_io_2d.cpp ${PROJECT_SOURCE_DIR}/test/test_constants.cpp)
ament_add_gtest(test_map_io_3d test_map_io_3d.cpp ${PROJECT_SOURCE_DIR}/test/test_constants.cpp)

ament_target_dependencies(test_map_io_2d rclcpp nav_msgs)
ament_target_dependencies(test_map_io_3d rclcpp nav_msgs
  nav2_msgs)

# costmap_filter_info_server unit test
ament_add_gtest(test_costmap_filter_info_server
  test_costmap_filter_info_server.cpp
)

ament_target_dependencies(test_costmap_filter_info_server rclcpp)

target_link_libraries(test_costmap_filter_info_server
  ${library_name}
)

target_link_libraries(test_map_io_2d
  ${map_io_library_name}_2d
  stdc++fs)

target_link_libraries(test_map_io_3d
  ${map_io_library_name}_3d
  stdc++fs)
