set(test_planner_costmaps_exec test_planner_costmaps_node)

ament_add_gtest_executable(${test_planner_costmaps_exec}
  test_planner_costmaps_node.cpp
  planner_tester.cpp
)
target_link_libraries(${test_planner_costmaps_exec}
  ${nav2_map_server_LIBRARIES}
  rclcpp::rclcpp
  nav2_util::nav2_util_core
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
  nav2_planner::planner_server_core
  nav2_map_server::map_io
)

set(test_planner_random_exec test_planner_random_node)

ament_add_gtest_executable(${test_planner_random_exec}
  test_planner_random_node.cpp
  planner_tester.cpp
)
target_link_libraries(${test_planner_random_exec}
  ${nav2_map_server_LIBRARIES}
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${nav2_msgs_TARGETS}
  tf2_ros::tf2_ros
  rclcpp_action::rclcpp_action
  ${visualization_msgs_TARGETS}
  nav2_planner::planner_server_core
  nav2_map_server::map_io
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
  test_planner_plugins.cpp
  TIMEOUT 10
)
target_link_libraries(test_planner_plugins
  stdc++fs
  ${nav_msgs_TARGETS}
  nav2_util::nav2_util_core
  nav2_core::nav2_core
  ${geometry_msgs_TARGETS}
  tf2::tf2
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  nav2_planner::planner_server_core
  ${visualization_msgs_TARGETS}
)

ament_add_gtest(test_planner_is_path_valid
  test_planner_is_path_valid.cpp
  planner_tester.cpp)
target_link_libraries(test_planner_is_path_valid
  rclcpp::rclcpp
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  nav2_planner::planner_server_core
  nav2_map_server::map_io
)
