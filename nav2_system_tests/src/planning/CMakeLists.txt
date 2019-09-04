ament_add_gtest_executable(test_planner_node
  test_planner_node.cpp
  planner_tester.cpp
)

ament_target_dependencies(test_planner_node
  ${dependencies}
)

ament_add_test(test_planner
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_planner_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_planner_node>
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map.pgm
)
