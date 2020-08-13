include_directories(${PROJECT_SOURCE_DIR}/test)

# map_server component test
ament_add_gtest_executable(test_map_server_node
  test_map_server_node.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)
ament_target_dependencies(test_map_server_node rclcpp nav_msgs)
target_link_libraries(test_map_server_node
  ${library_name}
  stdc++fs
)

ament_add_test(test_map_server_node
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_map_server_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_DIR=${TEST_DIR}
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_map_server_node>
)

# map_saver component test
ament_add_gtest_executable(test_map_saver_node
  test_map_saver_node.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)

ament_target_dependencies(test_map_saver_node rclcpp nav_msgs)
target_link_libraries(test_map_saver_node
  ${library_name}
  stdc++fs
)

add_executable(test_map_saver_publisher
  test_map_saver_publisher.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)

target_link_libraries(test_map_saver_publisher
  ${map_io_library_name}
  stdc++fs
)

ament_add_test(test_map_saver_node
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_map_saver_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_DIR=${TEST_DIR}
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_map_saver_node>
)
