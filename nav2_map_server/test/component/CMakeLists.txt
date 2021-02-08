include_directories(${PROJECT_SOURCE_DIR}/test)

# map_server component test
ament_add_gtest_executable(test_map_server_2d_node
  test_map_server_2d_node.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)

ament_target_dependencies(test_map_server_2d_node rclcpp nav_msgs nav2_msgs nav2_util)
target_link_libraries(test_map_server_2d_node
  stdc++fs
)

ament_add_test(test_map_server_2d_node
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_map_server_2d_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_DIR=${TEST_DIR}
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_map_server_2d_node>
)

# map_server_3d component test
ament_add_gtest_executable(test_map_server_3d_node
  test_map_server_3d_node.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)
ament_target_dependencies(test_map_server_3d_node rclcpp nav2_msgs
  nav2_util sensor_msgs geometry_msgs)
target_link_libraries(test_map_server_3d_node
  stdc++fs
)

ament_add_test(test_map_server_3d_node
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_map_server_3d_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_DIR=${TEST_DIR}
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_map_server_3d_node>
)

# map_saver component test
ament_add_gtest_executable(test_map_saver_2d_node
  test_map_saver_2d_node.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)

ament_target_dependencies(test_map_saver_2d_node rclcpp nav_msgs nav2_msgs nav2_util)
target_link_libraries(test_map_saver_2d_node
  ${library_name}
  stdc++fs
)

add_executable(test_map_saver_2d_publisher
  test_map_saver_2d_publisher.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)

target_link_libraries(test_map_saver_2d_publisher
  ${map_io_library_name}_2d
  stdc++fs
)

ament_add_test(test_map_saver_2d_node
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_map_saver_2d_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_DIR=${TEST_DIR}
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_map_saver_2d_node>
)

# map_saver component test for 3d saver
ament_add_gtest_executable(test_map_saver_3d_node
  test_map_saver_3d_node.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)

ament_target_dependencies(test_map_saver_3d_node rclcpp nav_msgs nav2_msgs
  nav2_util sensor_msgs geometry_msgs)
target_link_libraries(test_map_saver_3d_node
  ${library_name}
  stdc++fs
)

add_executable(test_map_saver_3d_publisher
  test_map_saver_3d_publisher.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)

target_link_libraries(test_map_saver_3d_publisher
  ${map_io_library_name}_3d
  stdc++fs
)

ament_add_test(test_map_saver_3d_node
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_map_saver_3d_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_DIR=${TEST_DIR}
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_map_saver_3d_node>
)
