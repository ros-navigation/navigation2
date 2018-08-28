ament_add_gtest_executable(test_dwa_controller_node
  test_dwa_controller_node.cpp
)
ament_target_dependencies(test_dwa_controller_node
  ${PROJECT_DEPENDENCIES}
)

ament_add_test(test_dwa_controller
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_dwa_controller_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_dwa_controller_node>
)
