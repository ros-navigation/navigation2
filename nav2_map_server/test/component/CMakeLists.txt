include_directories(${PROJECT_SOURCE_DIR}/test)

ament_add_gtest_executable(test_occ_grid_node
  test_occ_grid_node.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)
ament_target_dependencies(test_occ_grid_node rclcpp nav_msgs)
target_link_libraries(test_occ_grid_node
  ${library_name}
  stdc++fs
)

# occ_grid component test
ament_add_test(test_occ_grid_node
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_occ_grid_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_DIR=${TEST_DIR}
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_occ_grid_node>
)
