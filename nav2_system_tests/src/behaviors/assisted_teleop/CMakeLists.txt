set(test_assisted_teleop_behavior test_assisted_teleop_behavior_node)

ament_add_gtest_executable(${test_assisted_teleop_behavior}
  test_assisted_teleop_behavior_node.cpp
  assisted_teleop_behavior_tester.cpp
)

ament_target_dependencies(${test_assisted_teleop_behavior}
  ${dependencies}
)

ament_add_test(test_assisted_teleop_behavior
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_assisted_teleop_behavior_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:${test_assisted_teleop_behavior}>
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
)
