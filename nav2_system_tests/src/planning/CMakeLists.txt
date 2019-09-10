ament_add_gtest_executable(test_planner_costmaps_node
  test_planner_costmaps_node.cpp
  planner_tester.cpp
)

ament_target_dependencies(test_planner_costmaps_node
  ${dependencies}
)

ament_add_gtest_executable(test_planner_random_node
  test_planner_random_node.cpp
  planner_tester.cpp
)

ament_target_dependencies(test_planner_random_node
  ${dependencies}
)

ament_add_test(test_planner_costmaps
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_planner_costmaps_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_planner_costmaps_node>
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map.pgm
)

ament_add_test(test_planner_random
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_planner_random_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_planner_random_node>
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map.pgm
)
