ament_add_gtest(test_execution_timer test_execution_timer.cpp)

ament_add_gtest(test_node_utils test_node_utils.cpp)
target_link_libraries(test_node_utils ${library_name})

find_package(std_srvs)

ament_add_gtest(test_service_client test_service_client.cpp)
ament_target_dependencies(test_service_client std_srvs)
target_link_libraries(test_service_client ${library_name})

ament_add_gtest(test_string_utils test_string_utils.cpp)
target_link_libraries(test_string_utils ${library_name})

find_package(rclcpp_lifecycle REQUIRED)
ament_add_gtest(test_lifecycle_utils test_lifecycle_utils.cpp)
ament_target_dependencies(test_lifecycle_utils rclcpp_lifecycle)
target_link_libraries(test_lifecycle_utils ${library_name})

ament_add_gtest(test_actions test_actions.cpp)
ament_target_dependencies(test_actions rclcpp_action test_msgs)
target_link_libraries(test_actions ${library_name})

ament_add_gtest(test_lifecycle_node test_lifecycle_node.cpp)
ament_target_dependencies(test_lifecycle_node rclcpp_lifecycle)
target_link_libraries(test_lifecycle_node ${library_name})

ament_add_gtest(test_lifecycle_cli_node test_lifecycle_cli_node.cpp)
ament_target_dependencies(test_lifecycle_cli_node rclcpp_lifecycle)
target_link_libraries(test_lifecycle_cli_node ${library_name})

ament_add_gtest(test_geometry_utils test_geometry_utils.cpp)
ament_target_dependencies(test_geometry_utils geometry_msgs)
target_link_libraries(test_geometry_utils ${library_name})

ament_add_gtest(test_odometry_utils test_odometry_utils.cpp)
ament_target_dependencies(test_odometry_utils nav_msgs geometry_msgs)
target_link_libraries(test_odometry_utils ${library_name})

ament_add_gtest(test_robot_utils test_robot_utils.cpp)
ament_target_dependencies(test_robot_utils geometry_msgs)
target_link_libraries(test_robot_utils ${library_name})

# This test is disabled due to failing services
# https://github.com/ros-planning/navigation2/issues/1836

add_launch_test(
  "test_dump_params/test_dump_params_default.test.py"
  TARGET "test_dump_params_default"
  TIMEOUT 10
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:dump_params>
)

add_launch_test(
  "test_dump_params/test_dump_params_yaml.test.py"
  TARGET "test_dump_params_yaml"
  TIMEOUT 10
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:dump_params>
)

add_launch_test(
  "test_dump_params/test_dump_params_md.test.py"
  TARGET "test_dump_params_md"
  TIMEOUT 10
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:dump_params>
)

add_launch_test(
  "test_dump_params/test_dump_params_multiple.test.py"
  TARGET "test_dump_params_multiple"
  TIMEOUT 10
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:dump_params>
)
