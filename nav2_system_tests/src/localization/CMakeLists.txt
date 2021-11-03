ament_add_gtest_executable(test_localization_node
  test_localization_node.cpp
)
ament_target_dependencies(test_localization_node
  ${dependencies}
)


ament_add_test(test_localization
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_localization_launch.py"
  TIMEOUT 180
  ENV
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/warehouse.yaml
    TEST_EXECUTABLE=$<TARGET_FILE:test_localization_node>
    TEST_URDF=${PROJECT_SOURCE_DIR}/urdf/turtlebot3_waffle.urdf
    TEST_MODEL=${PROJECT_SOURCE_DIR}/worlds/waffle.model
)
