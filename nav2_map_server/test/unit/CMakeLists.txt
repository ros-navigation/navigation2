include_directories(${PROJECT_SOURCE_DIR}/test)

# map_io unit test
ament_add_gtest(test_map_io test_map_io.cpp ${PROJECT_SOURCE_DIR}/test/test_constants.cpp)
ament_add_gtest(test_map_io_3D test_map_io_3D.cpp ${PROJECT_SOURCE_DIR}/test/test_constants.cpp)

ament_target_dependencies(test_map_io rclcpp nav_msgs)
ament_target_dependencies(test_map_io_3D rclcpp nav_msgs
    sensor_msgs geometry_msgs)

target_link_libraries(test_map_io
  ${map_io_library_name}
  stdc++fs
)

# costmap_filter_info_server unit test
ament_add_gtest(test_costmap_filter_info_server
  test_costmap_filter_info_server.cpp
)

ament_target_dependencies(test_costmap_filter_info_server rclcpp)

target_link_libraries(test_costmap_filter_info_server
  ${library_name}
)

target_link_libraries(test_map_io_3D
    ${map_io_3D_library_name}
    stdc++fs)
