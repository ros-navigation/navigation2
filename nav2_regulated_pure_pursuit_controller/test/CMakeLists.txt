# tests for regulated PP
ament_add_gtest(test_regulated_pp
  test_regulated_pp.cpp
  path_utils/path_utils.cpp
)
target_link_libraries(test_regulated_pp
  ${library_name}
  rclcpp::rclcpp
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  nav2_core::nav2_core
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Path utils test
ament_add_gtest(test_path_utils path_utils/test_path_utils.cpp path_utils/path_utils.cpp)
target_link_libraries(test_path_utils ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs)
