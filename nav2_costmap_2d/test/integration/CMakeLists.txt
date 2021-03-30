ament_add_gtest_executable(footprint_tests_exec
  footprint_tests.cpp
)
ament_target_dependencies(footprint_tests_exec
  ${dependencies}
)
target_link_libraries(footprint_tests_exec
  nav2_costmap_2d_core
  layers
)

ament_add_gtest_executable(test_collision_checker_exec
  test_costmap_topic_collision_checker.cpp
)
ament_target_dependencies(test_collision_checker_exec
  ${dependencies}
)
target_link_libraries(test_collision_checker_exec
  nav2_costmap_2d_core
  nav2_costmap_2d_client
  layers
)

ament_add_gtest_executable(inflation_tests_exec
  inflation_tests.cpp
)
ament_target_dependencies(inflation_tests_exec
  ${dependencies}
)
target_link_libraries(inflation_tests_exec
  nav2_costmap_2d_core
  layers
)

ament_add_gtest_executable(obstacle_tests_exec
  obstacle_tests.cpp
)
ament_target_dependencies(obstacle_tests_exec
  ${dependencies}
)
target_link_libraries(obstacle_tests_exec
  nav2_costmap_2d_core
  layers
)

ament_add_gtest_executable(range_tests_exec
  range_tests.cpp
)
ament_target_dependencies(range_tests_exec
  ${dependencies}
)
target_link_libraries(range_tests_exec
  nav2_costmap_2d_core
  layers
)

ament_add_test(test_collision_checker
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/costmap_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_MAP=${TEST_MAP_DIR}/TenByTen.yaml
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_collision_checker_exec>
)

ament_add_test(footprint_tests
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/costmap_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_MAP=${TEST_MAP_DIR}/TenByTen.yaml
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:footprint_tests_exec>
)

ament_add_test(inflation_tests
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/costmap_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_MAP=${TEST_MAP_DIR}/TenByTen.yaml
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:inflation_tests_exec>
)

ament_add_test(obstacle_tests
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/costmap_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_MAP=${TEST_MAP_DIR}/TenByTen.yaml
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:obstacle_tests_exec>
)

ament_add_test(range_tests
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/costmap_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_MAP=${TEST_MAP_DIR}/TenByTen.yaml
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:range_tests_exec>
)

## TODO(bpwilcox): this test (I believe) is intended to be launched with the simple_driving_test.xml,
## which has a dependency on rosbag playback
# ament_add_gtest_executable(costmap_tester
#   costmap_tester.cpp
# )
# ament_target_dependencies(costmap_tester
#   ${dependencies}
# )
# target_link_libraries(costmap_tester
#   nav2_costmap_2d_core
#   layers
# )
