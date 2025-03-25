find_package(nav2_controller REQUIRED)

# Tests for Graceful Controller
ament_add_gtest(test_graceful_controller
  test_graceful_controller.cpp
)
ament_target_dependencies(test_graceful_controller
  geometry_msgs
  nav2_controller
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  rclcpp
  tf2_ros
  visualization_msgs
)
target_link_libraries(test_graceful_controller
  ${library_name}
)
# target_link_libraries(test_graceful_controller
#   ${library_name}
#   ${geometry_msgs_TARGETS}
#   nav2_controller::simple_goal_checker
#   nav2_core::nav2_core
#   nav2_costmap_2d::nav2_costmap_2d_core
#   nav2_util::nav2_util_core
#   ${nav_msgs_TARGETS}
#   rclcpp::rclcpp
#   tf2_ros::tf2_ros
#   ${visualization_msgs_TARGETS}
# )

# Egopolar test
ament_add_gtest(test_egopolar
  test_egopolar.cpp
)
ament_target_dependencies(test_egopolar
  geometry_msgs
  rclcpp
  tf2_geometry_msgs
)
target_link_libraries(test_egopolar
  ${library_name}
)
# target_link_libraries(test_egopolar
#   ${library_name}
#   ${geometry_msgs_TARGETS}
#   rclcpp::rclcpp
#   tf2_geometry_msgs::tf2_geometry_msgs
# )
