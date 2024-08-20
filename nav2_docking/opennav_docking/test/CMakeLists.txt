# Test utils
ament_add_gtest(test_utils
  test_utils.cpp
)
target_link_libraries(test_utils
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test dock database
ament_add_gtest(test_dock_database
  test_dock_database.cpp
)
target_link_libraries(test_dock_database
  ament_index_cpp::ament_index_cpp
  ${library_name}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test navigator
ament_add_gtest(test_navigator
  test_navigator.cpp
)
target_link_libraries(test_navigator
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  ${library_name}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test Controller
ament_add_gtest(test_controller
  test_controller.cpp
)
target_link_libraries(test_controller
  ${geometry_msgs_TARGETS}
  ${library_name}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test Simple Dock
ament_add_gtest(test_simple_charging_dock
  test_simple_charging_dock.cpp
)
target_link_libraries(test_simple_charging_dock
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  simple_charging_dock
  tf2_geometry_msgs::tf2_geometry_msgs
)

# Test Pose Filter (unit)
ament_add_gtest(test_pose_filter
  test_pose_filter.cpp
)
target_link_libraries(test_pose_filter
  ${geometry_msgs_TARGETS}
  pose_filter
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# Test dock pluing for server tests
add_library(test_dock SHARED testing_dock.cpp)
target_link_libraries(test_dock PUBLIC
  ${geometry_msgs_TARGETS}
  opennav_docking_core::opennav_docking_core
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
)
install(TARGETS test_dock
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_export_libraries(test_dock)

# Test docking server (unit)
ament_add_gtest(test_docking_server_unit
  test_docking_server_unit.cpp
)
target_link_libraries(test_docking_server_unit
  ${geometry_msgs_TARGETS}
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test docking server (smoke)
ament_add_pytest_test("test_docking_server" test_docking_server.py)
