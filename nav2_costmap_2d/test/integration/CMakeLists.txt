ament_add_gtest_executable(footprint_tests_exec
  footprint_tests.cpp
)
target_link_libraries(footprint_tests_exec
  ${PROJECT_NAME}::nav2_costmap_2d_core
  ${PROJECT_NAME}::layers
)

ament_add_gtest_executable(test_collision_checker_exec
  test_costmap_topic_collision_checker.cpp
)
target_link_libraries(test_collision_checker_exec
  ${PROJECT_NAME}::nav2_costmap_2d_core
  ${PROJECT_NAME}::nav2_costmap_2d_client
  ${PROJECT_NAME}::layers
)

ament_add_gtest_executable(inflation_tests_exec
  inflation_tests.cpp
)
target_link_libraries(inflation_tests_exec
  ${PROJECT_NAME}::nav2_costmap_2d_core
  ${PROJECT_NAME}::layers
)

ament_add_gtest_executable(obstacle_tests_exec
  obstacle_tests.cpp
)
target_link_libraries(obstacle_tests_exec
  ${PROJECT_NAME}::nav2_costmap_2d_core
  ${PROJECT_NAME}::layers
)

ament_add_gtest_executable(range_tests_exec
  range_tests.cpp
)
target_link_libraries(range_tests_exec
  ${PROJECT_NAME}::nav2_costmap_2d_core
  ${PROJECT_NAME}::layers
)

ament_add_gtest_executable(plugin_container_tests_exec
  plugin_container_tests.cpp
)
target_link_libraries(plugin_container_tests_exec
  nav2_costmap_2d_core
  layers
)


ament_add_gtest(dyn_params_tests
  dyn_params_tests.cpp
)
target_link_libraries(dyn_params_tests
  ${PROJECT_NAME}::nav2_costmap_2d_core
)

ament_add_gtest_executable(test_costmap_publisher_exec
    test_costmap_2d_publisher.cpp
)
target_link_libraries(test_costmap_publisher_exec
    ${PROJECT_NAME}::nav2_costmap_2d_core
    ${PROJECT_NAME}::nav2_costmap_2d_client
    ${PROJECT_NAME}::layers
)

ament_add_gtest_executable(test_costmap_subscriber_exec
test_costmap_subscriber.cpp
)
target_link_libraries(test_costmap_subscriber_exec
    ${PROJECT_NAME}::nav2_costmap_2d_core
    ${PROJECT_NAME}::nav2_costmap_2d_client
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

ament_add_test(plugin_container_tests
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/costmap_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_MAP=${TEST_MAP_DIR}/TenByTen.yaml
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:plugin_container_tests_exec>
)

ament_add_test(test_costmap_publisher_exec
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/costmap_tests_launch.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    ENV
    TEST_MAP=${TEST_MAP_DIR}/TenByTen.yaml
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_costmap_publisher_exec>
)

ament_add_test(test_costmap_subscriber_exec
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/costmap_tests_launch.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    ENV
    TEST_MAP=${TEST_MAP_DIR}/TenByTen.yaml
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_costmap_subscriber_exec>
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
#   ${PROJECT_NAME}::nav2_costmap_2d_core
#   layers
# )
