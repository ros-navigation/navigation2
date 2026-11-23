# tests for regulated PP
find_package(nav2_controller REQUIRED)

nav2_add_gtest(test_regulated_pp
  test_regulated_pp.cpp
  test_dwpp_functions.cpp
  path_utils/path_utils.cpp
)
target_link_libraries(test_regulated_pp
  ${library_name}
  geometry_msgs::geometry_msgs
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  nav2_controller::feasible_path_handler
  nav2_controller::simple_goal_checker
  nav_msgs::nav_msgs
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Path utils test
nav2_add_gtest(test_path_utils path_utils/test_path_utils.cpp path_utils/path_utils.cpp)
target_link_libraries(test_path_utils
  nav_msgs::nav_msgs
  geometry_msgs::geometry_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# Latency compensator unit tests
nav2_add_gtest(test_latency_compensator test_latency_compensator.cpp)
target_link_libraries(test_latency_compensator
  ${library_name}
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
)

# Closed-loop benchmark on a robot with latency, lag, gain error and deadbands
nav2_add_gtest(test_latency_benchmark
  test_latency_benchmark.cpp
  path_utils/path_utils.cpp
  TIMEOUT 600
)
target_link_libraries(test_latency_benchmark
  ${library_name}
  angles::angles
  geometry_msgs::geometry_msgs
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  nav2_controller::simple_goal_checker
  nav_msgs::nav_msgs
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
)
