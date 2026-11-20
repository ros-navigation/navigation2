# Cancel test
nav2_add_gtest(test_dock_robot test_dock_robot.cpp)
target_link_libraries(test_dock_robot
  opennav_dock_action_bt_node
  behaviortree_cpp::behaviortree_cpp
  geometry_msgs::geometry_msgs
  nav2_msgs::nav2_msgs
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

# Make command test
nav2_add_gtest(test_undock_robot test_undock_robot.cpp)
target_link_libraries(test_undock_robot
  opennav_undock_action_bt_node
  behaviortree_cpp::behaviortree_cpp
  geometry_msgs::geometry_msgs
  nav2_msgs::nav2_msgs
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)
