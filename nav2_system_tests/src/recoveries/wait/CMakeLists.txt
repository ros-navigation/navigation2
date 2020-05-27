set(test_wait_recovery_exec test_wait_recovery_node)

ament_add_gtest_executable(${test_wait_recovery_exec}
  test_wait_recovery_node.cpp
  wait_recovery_tester.cpp
)

ament_target_dependencies(${test_wait_recovery_exec}
  ${dependencies}
)

ament_add_test(test_wait_recovery
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_wait_recovery_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map_circular.yaml
    TEST_EXECUTABLE=$<TARGET_FILE:${test_wait_recovery_exec}>
    TEST_WORLD=${PROJECT_SOURCE_DIR}/worlds/turtlebot3_ros2_demo.world
    GAZEBO_MODEL_PATH=${PROJECT_SOURCE_DIR}/models
    BT_NAVIGATOR_XML=navigate_w_replanning_and_recovery.xml
)
