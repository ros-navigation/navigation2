set(test_drive_on_heading_behavior test_drive_on_heading_behavior_node)

ament_add_gtest_executable(${test_drive_on_heading_behavior}
  test_drive_on_heading_behavior_node.cpp
  drive_on_heading_behavior_tester.cpp
)
target_link_libraries(${test_drive_on_heading_behavior}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  angles::angles
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)

ament_add_test(test_drive_on_heading_recovery
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_drive_on_heading_behavior_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map_circular.yaml
    TEST_EXECUTABLE=$<TARGET_FILE:${test_drive_on_heading_behavior}>
    TEST_WORLD=${PROJECT_SOURCE_DIR}/worlds/turtlebot3_ros2_demo.world
    GAZEBO_MODEL_PATH=${PROJECT_SOURCE_DIR}/models
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
)
