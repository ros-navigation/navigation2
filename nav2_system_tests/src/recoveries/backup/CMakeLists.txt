set(test_backup_recovery_exec test_backup_recovery_node)

ament_add_gtest_executable(${test_backup_recovery_exec}
  test_backup_recovery_node.cpp
  backup_recovery_tester.cpp
)

ament_target_dependencies(${test_backup_recovery_exec}
  ${dependencies}
)

ament_add_test(test_backup_recovery
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_backup_recovery_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:${test_backup_recovery_exec}>
    TEST_URDF=${PROJECT_SOURCE_DIR}/urdf/turtlebot3_waffle.urdf
    TEST_MODEL=${PROJECT_SOURCE_DIR}/worlds/waffle.model
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
)
