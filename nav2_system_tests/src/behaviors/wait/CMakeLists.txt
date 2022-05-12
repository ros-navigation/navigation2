set(test_wait_behavior_exec test_wait_behavior_node)

ament_add_gtest_executable(${test_wait_behavior_exec}
  test_wait_behavior_node.cpp
  wait_behavior_tester.cpp
)

ament_target_dependencies(${test_wait_behavior_exec}
  ${dependencies}
)

ament_add_test(test_wait_behavior
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_wait_behavior_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:${test_wait_behavior_exec}>
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_behavior.xml
)
