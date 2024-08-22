ament_add_gtest_executable(test_localization_node
  test_localization_node.cpp
)
target_link_libraries(test_localization_node
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
)

ament_add_test(test_localization
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_localization_launch.py"
  TIMEOUT 180
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:test_localization_node>
)
