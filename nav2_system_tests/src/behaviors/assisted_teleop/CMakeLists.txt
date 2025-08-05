set(test_assisted_teleop_behavior test_assisted_teleop_behavior_node)

ament_add_gtest_executable(${test_assisted_teleop_behavior}
  test_assisted_teleop_behavior_node.cpp
  assisted_teleop_behavior_tester.cpp
)
target_link_libraries(${test_assisted_teleop_behavior}
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  ${nav2_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  nav2_ros_common::nav2_ros_common
)
target_include_directories(${test_assisted_teleop_behavior}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
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
