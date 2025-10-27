# Cancel test
ament_add_gtest(test_dock_robot test_dock_robot.cpp)
target_link_libraries(test_dock_robot
  behaviortree_cpp::behaviortree_cpp
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  opennav_dock_action_bt_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

# Make command test
ament_add_gtest(test_undock_robot test_undock_robot.cpp)
target_link_libraries(test_undock_robot
  behaviortree_cpp::behaviortree_cpp
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  opennav_undock_action_bt_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)
