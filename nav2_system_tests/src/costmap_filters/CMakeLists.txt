ament_add_test(test_keepout_filter
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_keepout_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TEST_MASK=${PROJECT_SOURCE_DIR}/maps/keepout_mask.yaml
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
    ASTAR=False
)

ament_add_test(test_speed_filter_global
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_speed_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TEST_MASK=${PROJECT_SOURCE_DIR}/maps/speed_mask.yaml
    PARAMS_FILE=${CMAKE_CURRENT_SOURCE_DIR}/speed_global_params.yaml
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
    ASTAR=False
)

ament_add_test(test_speed_filter_local
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_speed_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TEST_MASK=${PROJECT_SOURCE_DIR}/maps/speed_mask.yaml
    PARAMS_FILE=${CMAKE_CURRENT_SOURCE_DIR}/speed_local_params.yaml
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
    ASTAR=False
)
