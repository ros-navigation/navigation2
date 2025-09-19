ament_add_test(test_failure_navigator
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_failure_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
)
