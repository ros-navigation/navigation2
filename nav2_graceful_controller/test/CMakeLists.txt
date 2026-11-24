find_package(nav2_controller REQUIRED)

# Tests for Graceful Controller
nav2_add_gtest(test_graceful_controller
  test_graceful_controller.cpp
)
target_link_libraries(test_graceful_controller
  ${library_name}
  geometry_msgs::geometry_msgs
  nav2_controller::simple_goal_checker
  nav2_controller::feasible_path_handler
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  nav_msgs::nav_msgs
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  visualization_msgs::visualization_msgs
)

# Egopolar test
nav2_add_gtest(test_egopolar
  test_egopolar.cpp
)
target_link_libraries(test_egopolar
  ${library_name}
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
)
