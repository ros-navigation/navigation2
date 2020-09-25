find_package(nav2_costmap_2d REQUIRED)

set(test_keepout_filter_exec test_keepout_filter)

ament_add_gtest_executable(${test_keepout_filter_exec}
  test_keepout_filter.cpp filters_tester.cpp
)
ament_target_dependencies(${test_keepout_filter_exec}
  ${dependencies}
  nav2_costmap_2d
)

ament_add_test(test_keepout_filter
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/costmap_filters_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 120
  ENV
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/filters_room.yaml
    TEST_MASK=${PROJECT_SOURCE_DIR}/maps/keepout_mask.yaml
    FILTER_PARAMS=${PROJECT_SOURCE_DIR}/params/keepout_params.yaml
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:${test_keepout_filter_exec}>
)
