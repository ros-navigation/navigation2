ament_add_gtest(test_execution_timer test_execution_timer.cpp)
target_link_libraries(test_execution_timer ${library_name})

ament_add_gtest(test_node_utils test_node_utils.cpp)
target_link_libraries(test_node_utils ${library_name})

find_package(std_srvs REQUIRED)
find_package(test_msgs REQUIRED)

ament_add_gtest(test_service_client test_service_client.cpp)
target_link_libraries(test_service_client ${library_name} ${std_srvs_TARGETS})

ament_add_gtest(test_string_utils test_string_utils.cpp)
target_link_libraries(test_string_utils ${library_name})

find_package(rclcpp_lifecycle REQUIRED)
ament_add_gtest(test_lifecycle_utils test_lifecycle_utils.cpp)
target_link_libraries(test_lifecycle_utils ${library_name} rclcpp_lifecycle::rclcpp_lifecycle)

ament_add_gtest(test_actions test_actions.cpp)
target_link_libraries(test_actions ${library_name} rclcpp_action::rclcpp_action ${test_msgs_TARGETS})

ament_add_gtest(test_lifecycle_node test_lifecycle_node.cpp)
target_link_libraries(test_lifecycle_node ${library_name} rclcpp_lifecycle::rclcpp_lifecycle)

ament_add_gtest(test_lifecycle_cli_node test_lifecycle_cli_node.cpp)
target_link_libraries(test_lifecycle_cli_node ${library_name} rclcpp_lifecycle::rclcpp_lifecycle)

ament_add_gtest(test_geometry_utils test_geometry_utils.cpp)
target_link_libraries(test_geometry_utils ${library_name} ${geometry_msgs_TARGETS})

ament_add_gtest(test_odometry_utils test_odometry_utils.cpp)
target_link_libraries(test_odometry_utils ${library_name} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS})

ament_add_gtest(test_robot_utils test_robot_utils.cpp)
target_link_libraries(test_robot_utils ${library_name} ${geometry_msgs_TARGETS})

ament_add_gtest(test_base_footprint_publisher test_base_footprint_publisher.cpp)
target_include_directories(test_base_footprint_publisher PRIVATE "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>")

target_link_libraries(test_base_footprint_publisher ${library_name} tf2_ros::tf2_ros rclcpp::rclcpp ${geometry_msgs_TARGETS})

ament_add_gtest(test_array_parser test_array_parser.cpp)
target_link_libraries(test_array_parser ${library_name})

ament_add_gtest(test_twist_publisher test_twist_publisher.cpp)
target_link_libraries(test_twist_publisher ${library_name} rclcpp::rclcpp ${geometry_msgs_TARGETS})

ament_add_gtest(test_twist_subscriber test_twist_subscriber.cpp)
target_link_libraries(test_twist_subscriber ${library_name} rclcpp::rclcpp ${geometry_msgs_TARGETS})

ament_add_gtest(test_validation_messages test_validation_messages.cpp)
target_link_libraries(test_validation_messages ${library_name} ${builtin_interfaces_TARGETS} ${std_msgs_TARGETS} ${geometry_msgs_TARGETS})
