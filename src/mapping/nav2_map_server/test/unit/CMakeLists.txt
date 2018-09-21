include_directories(${PROJECT_SOURCE_DIR}/test)

ament_add_gtest(test_occ_grid test_occ_grid.cpp ${PROJECT_SOURCE_DIR}/test/test_constants.cpp)

ament_target_dependencies(test_occ_grid rclcpp nav_msgs)

target_link_libraries(test_occ_grid
  ${library_name}
  stdc++fs
)

