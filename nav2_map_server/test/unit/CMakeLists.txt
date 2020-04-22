include_directories(${PROJECT_SOURCE_DIR}/test)

# map_io unit test
ament_add_gtest(test_map_io test_map_io.cpp ${PROJECT_SOURCE_DIR}/test/test_constants.cpp)

ament_target_dependencies(test_map_io rclcpp nav_msgs)

target_link_libraries(test_map_io
  ${map_io_library_name}
  stdc++fs
)
