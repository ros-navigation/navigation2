set(test_wait_behavior_exec test_wait_behavior_node)

ament_add_gtest_executable(${test_wait_behavior_exec}
  test_wait_behavior_node.cpp
  wait_behavior_tester.cpp
)
target_link_libraries(${test_wait_behavior_exec}
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2::tf2
  tf2_ros::tf2_ros
  nav2_ros_common::nav2_ros_common
)
target_include_directories(${test_wait_behavior_exec}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
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
