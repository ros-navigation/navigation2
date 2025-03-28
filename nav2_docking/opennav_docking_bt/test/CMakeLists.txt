# Cancel test
ament_add_gtest(test_dock_robot test_dock_robot.cpp)
ament_target_dependencies(test_dock_robot
  behaviortree_cpp
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
)
target_link_libraries(test_dock_robot
  opennav_dock_action_bt_node
)

# Make command test
ament_add_gtest(test_undock_robot test_undock_robot.cpp)
ament_target_dependencies(test_dock_robot
  behaviortree_cpp
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
)
target_link_libraries(test_undock_robot
  opennav_undock_action_bt_node
)
