set(test_wait_behavior_exec test_wait_behavior_node)

ament_add_gtest_executable(${test_wait_behavior_exec}
  test_wait_behavior_node.cpp
  wait_behavior_tester.cpp
)
target_link_libraries(${test_wait_behavior_exec}
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  rclcpp_action::rclcpp_action
  angles::angles
  nav2_util::nav2_util_core
  ${nav2_msgs_TARGETS}
)

ament_add_test(test_wait_behavior
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_wait_behavior_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map_circular.yaml
    TEST_EXECUTABLE=$<TARGET_FILE:${test_wait_behavior_exec}>
    TEST_WORLD=${PROJECT_SOURCE_DIR}/worlds/turtlebot3_ros2_demo.world
    GAZEBO_MODEL_PATH=${PROJECT_SOURCE_DIR}/models
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_behavior.xml
)
