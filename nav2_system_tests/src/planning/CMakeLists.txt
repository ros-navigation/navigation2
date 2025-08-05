set(test_planner_costmaps_exec test_planner_costmaps_node)

ament_add_gtest_executable(${test_planner_costmaps_exec}
  test_planner_costmaps_node.cpp
  planner_tester.cpp
)
target_link_libraries(${test_planner_costmaps_exec}
  ${geometry_msgs_TARGETS}
  nav2_map_server::map_io
  nav2_map_server::map_server_core
  ${nav2_msgs_TARGETS}
  nav2_planner::planner_server_core
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
  nav2_ros_common::nav2_ros_common
)
target_include_directories(${test_planner_costmaps_exec}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

set(test_planner_random_exec test_planner_random_node)

ament_add_gtest_executable(${test_planner_random_exec}
  test_planner_random_node.cpp
  planner_tester.cpp
)
target_link_libraries(${test_planner_random_exec}
  ${geometry_msgs_TARGETS}
  nav2_map_server::map_io
  nav2_map_server::map_server_core
  ${nav2_msgs_TARGETS}
  nav2_planner::planner_server_core
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
  nav2_ros_common::nav2_ros_common
)
target_include_directories(${test_planner_random_exec}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_add_test(test_planner_costmaps
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_planner_costmaps_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:${test_planner_costmaps_exec}>
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map.pgm
)

ament_add_test(test_planner_random
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_planner_random_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:${test_planner_random_exec}>
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map.pgm
)

ament_add_gtest(test_planner_plugins
  planner_tester.cpp
  test_planner_plugins.cpp
  TIMEOUT 10
)
target_link_libraries(test_planner_plugins
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_map_server::map_io
  nav2_map_server::map_server_core
  ${nav2_msgs_TARGETS}
  nav2_planner::planner_server_core
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  nav2_ros_common::nav2_ros_common
)
target_include_directories(test_planner_plugins
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_add_gtest(test_planner_is_path_valid
  planner_tester.cpp
  test_planner_is_path_valid.cpp
)
target_link_libraries(test_planner_is_path_valid
  ${geometry_msgs_TARGETS}
  nav2_map_server::map_io
  nav2_map_server::map_server_core
  ${nav2_msgs_TARGETS}
  nav2_planner::planner_server_core
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  nav2_ros_common::nav2_ros_common
)
target_include_directories(test_planner_is_path_valid
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
