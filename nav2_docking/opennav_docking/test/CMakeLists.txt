# Test utils
ament_add_gtest(test_utils
  test_utils.cpp
)
ament_target_dependencies(test_utils
  ament_index_cpp
  geometry_msgs
  nav2_util
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(test_utils
  ${library_name}
)

# Test dock database
ament_add_gtest(test_dock_database
  test_dock_database.cpp
)
ament_target_dependencies(test_dock_database
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(test_dock_database
  ${library_name}
)

# Test navigator
ament_add_gtest(test_navigator
  test_navigator.cpp
)
ament_target_dependencies(test_navigator
  ament_index_cpp
  geometry_msgs
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(test_navigator
  ${library_name}
)

# Test Controller
ament_add_gtest(test_controller
  test_controller.cpp
)
ament_target_dependencies(test_controller
  geometry_msgs
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(test_controller
  ${library_name}
)

# Test Simple Dock
ament_add_gtest(test_simple_charging_dock
  test_simple_charging_dock.cpp
)
ament_target_dependencies(test_simple_charging_dock
  ament_index_cpp
  geometry_msgs
  sensor_msgs
  nav2_util
  rclcpp
  rclcpp_lifecycle
  tf2_geometry_msgs
)
target_link_libraries(test_simple_charging_dock
  simple_charging_dock
)
ament_add_gtest(test_simple_non_charging_dock
  test_simple_non_charging_dock.cpp
)
ament_target_dependencies(test_simple_non_charging_dock
  ${dependencies}
)
target_link_libraries(test_simple_non_charging_dock
  ${library_name} simple_non_charging_dock
)

# Test Pose Filter (unit)
ament_add_gtest(test_pose_filter
  test_pose_filter.cpp
)
ament_target_dependencies(test_pose_filter
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)
target_link_libraries(test_pose_filter
  pose_filter
)

# Test dock pluing for server tests
add_library(test_dock SHARED testing_dock.cpp)
ament_target_dependencies(test_dock
  geometry_msgs
  opennav_docking_core
  pluginlib
  rclcpp_lifecycle
  tf2_ros
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
ament_target_dependencies(test_docking_server_unit
  geometry_msgs
  nav2_util
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(test_docking_server_unit
  ${library_name}
)

# Test docking server (smoke)
ament_add_pytest_test("test_docking_server" test_docking_server.py)
