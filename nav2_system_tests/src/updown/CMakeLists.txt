ament_add_gtest_executable(test_updown
  test_updown.cpp
)

ament_target_dependencies(test_updown
  ${dependencies}
)

ament_add_test(test_updown
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/start_nav2"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  TIMEOUT 100
  ENV
    TEST_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}
)
