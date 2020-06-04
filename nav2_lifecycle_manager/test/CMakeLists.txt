add_executable(example_lifecycle_node lifecycle_node_test.cpp)

ament_target_dependencies(example_lifecycle_node
  ${dependencies}
)

ament_add_gtest_executable(test_lifecycle_gtest
  test_lifecycle_manager.cpp
)

ament_target_dependencies(test_lifecycle_gtest
  ${dependencies}
)

ament_add_test(test_lifecycle
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_lifecycle_manager.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:test_lifecycle_gtest>
    EXAMPLE_EXECUTABLE=$<TARGET_FILE:example_lifecycle_node>
)
