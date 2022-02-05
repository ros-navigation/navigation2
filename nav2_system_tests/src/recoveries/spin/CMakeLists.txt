set(test_spin_recovery_exec test_spin_recovery_node)

ament_add_gtest_executable(${test_spin_recovery_exec}
  test_spin_recovery_node.cpp
  spin_recovery_tester.cpp
)

ament_target_dependencies(${test_spin_recovery_exec}
  ${dependencies}
)

ament_add_test(test_spin_recovery
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_spin_recovery_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:${test_spin_recovery_exec}>
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
)

ament_add_test(test_spin_recovery_fake
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_spin_recovery_fake_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:${test_spin_recovery_exec}>
    MAKE_FAKE_COSTMAP=true
)
