# tests
find_package(nav2_controller REQUIRED)

ament_add_gtest(test_shim_controller
  test_shim_controller.cpp
)
target_link_libraries(test_shim_controller
  ${library_name}
  rclcpp::rclcpp
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  nav2_controller::simple_goal_checker
  tf2_ros::tf2_ros
)
