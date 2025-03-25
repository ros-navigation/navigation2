# Test test executors
ament_add_gtest(test_task_executors
  test_task_executors.cpp
)
ament_target_dependencies(test_task_executors
  geometry_msgs
  nav2_util
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)
target_link_libraries(test_task_executors
  ${library_name}
  wait_at_waypoint
  photo_at_waypoint
  input_at_waypoint
  # ${geometry_msgs_TARGETS}
  # nav2_util::nav2_util_core
  # rclcpp::rclcpp
  # rclcpp_lifecycle::rclcpp_lifecycle
  # ${sensor_msgs_TARGETS}
)

# Test dynamic parameters
ament_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
ament_target_dependencies(test_dynamic_parameters
  nav2_util
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
