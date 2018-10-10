include_directories( ${PROJECT_SOURCE_DIR}/src )

ament_add_gtest_executable(test_localization_node
  test_localization_node.cpp
)
ament_target_dependencies(test_localization_node
  ${dependencies}
)

ament_add_test(test_localization
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_localization_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 20
  ENV
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_localization_node>
)

