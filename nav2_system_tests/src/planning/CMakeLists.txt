set(test_planner_costmaps_exec test_planner_costmaps_node)

ament_add_gtest_executable(${test_planner_costmaps_exec}
  test_planner_costmaps_node.cpp
  planner_tester.cpp
)

target_link_libraries(${test_planner_costmaps_exec}
  ${nav2_map_server_LIBRARIES})

ament_target_dependencies(${test_planner_costmaps_exec}
  ${dependencies}
)

set(test_planner_random_exec test_planner_random_node)

ament_add_gtest_executable(${test_planner_random_exec}
  test_planner_random_node.cpp
  planner_tester.cpp
)

ament_target_dependencies(${test_planner_random_exec}
  ${dependencies}
)

target_link_libraries(${test_planner_random_exec}
  ${nav2_map_server_LIBRARIES})

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

ament_target_dependencies(test_planner_plugins rclcpp geometry_msgs nav2_msgs ${dependencies})

ament_add_gtest(test_planner_is_path_valid
              test_planner_is_path_valid.cpp
              planner_tester.cpp)

ament_target_dependencies(test_planner_is_path_valid rclcpp geometry_msgs nav2_msgs ${dependencies})
