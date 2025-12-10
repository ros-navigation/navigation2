# tests for regulated PP
find_package(nav2_controller REQUIRED)

nav2_add_gtest(test_regulated_pp
  test_regulated_pp.cpp
  test_dwpp_functions.cpp
  path_utils/path_utils.cpp
)
target_link_libraries(test_regulated_pp
  ${library_name}
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  nav2_controller::simple_goal_checker
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Path utils test
nav2_add_gtest(test_path_utils path_utils/test_path_utils.cpp path_utils/path_utils.cpp)
target_link_libraries(test_path_utils
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)
