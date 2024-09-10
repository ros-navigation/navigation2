ament_add_gtest(prefer_forward_tests prefer_forward_test.cpp)
target_link_libraries(prefer_forward_tests
  dwb_critics
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  rclcpp::rclcpp
)

ament_add_gtest(base_obstacle_tests base_obstacle_test.cpp)
target_link_libraries(base_obstacle_tests
  dwb_critics
  dwb_core::dwb_core
  rclcpp::rclcpp
)

ament_add_gtest(obstacle_footprint_tests obstacle_footprint_test.cpp)
target_link_libraries(obstacle_footprint_tests
  dwb_critics
  dwb_core::dwb_core
  rclcpp::rclcpp
)

ament_add_gtest(alignment_util_tests alignment_util_test.cpp)
target_link_libraries(alignment_util_tests
  dwb_critics
  dwb_core::dwb_core
  rclcpp::rclcpp
)

ament_add_gtest(twirling_tests twirling_test.cpp)
target_link_libraries(twirling_tests
  dwb_critics
  dwb_core::dwb_core
  rclcpp::rclcpp
)
