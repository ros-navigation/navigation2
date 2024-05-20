# Kinematics test
ament_add_gtest(kinematics_test kinematics_test.cpp)
target_link_libraries(kinematics_test
  rclcpp::rclcpp
  ${monitor_library_name}
)

# Data sources test
ament_add_gtest(sources_test sources_test.cpp)
target_link_libraries(sources_test
  ${monitor_library_name}
  rclcpp::rclcpp
  nav2_util::nav2_util_core
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
)

# Polygon shapes test
ament_add_gtest(polygons_test polygons_test.cpp)
target_link_libraries(polygons_test
  ${monitor_library_name}
  rclcpp::rclcpp
  nav2_util::nav2_util_core
  ${geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
)

# Collision Monitor node test
ament_add_gtest(collision_monitor_node_test collision_monitor_node_test.cpp)
target_link_libraries(collision_monitor_node_test
  ${monitor_library_name}
  rclcpp::rclcpp
  nav2_util::nav2_util_core
  ${nav2_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2_ros::tf2_ros
)
# Collision Detector node test
ament_add_gtest(collision_detector_node_test collision_detector_node_test.cpp)
target_link_libraries(collision_detector_node_test
  ${detector_library_name}
  rclcpp::rclcpp
  nav2_util::nav2_util_core
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2_ros::tf2_ros
)
