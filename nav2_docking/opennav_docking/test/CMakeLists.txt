# Test utils
ament_add_gtest(test_utils
  test_utils.cpp
)
target_link_libraries(test_utils
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  ${library_name}
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_include_directories(test_utils
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
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
  nav2_ros_common::nav2_ros_common
)
target_include_directories(test_dock_database
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
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
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_include_directories(test_navigator
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
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
  nav2_ros_common::nav2_ros_common
)
target_include_directories(test_controller
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Test Simple Dock
ament_add_gtest(test_simple_charging_dock
  test_simple_charging_dock.cpp
)
target_link_libraries(test_simple_charging_dock
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  simple_charging_dock
  tf2_geometry_msgs::tf2_geometry_msgs
)
target_include_directories(test_simple_charging_dock
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_add_gtest(test_simple_non_charging_dock
  test_simple_non_charging_dock.cpp
)
target_link_libraries(test_simple_non_charging_dock
  ${geometry_msgs_TARGETS}
  ${library_name}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  nav2_ros_common::nav2_ros_common
)
target_link_libraries(test_simple_non_charging_dock
  ${library_name} simple_non_charging_dock
)
target_include_directories(test_simple_non_charging_dock
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Test Pose Filter (unit)
ament_add_gtest(test_pose_filter
  test_pose_filter.cpp
)
target_link_libraries(test_pose_filter
  ${geometry_msgs_TARGETS}
  nav2_ros_common::nav2_ros_common
  pose_filter
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)
target_include_directories(test_pose_filter
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Test dock pluing for server tests
add_library(test_dock SHARED testing_dock.cpp)
target_link_libraries(test_dock PUBLIC
  ${geometry_msgs_TARGETS}
  ${bond_TARGETS}
  opennav_docking_core::opennav_docking_core
  nav2_ros_common::nav2_ros_common
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  bondcpp::bondcpp
)
install(TARGETS test_dock
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
target_include_directories(test_dock
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
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
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_include_directories(test_docking_server_unit
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Test docking server (smoke)
ament_add_pytest_test(test_docking_server_with_charging_dock test_docking_server.py
  ENV
  NON_CHARGING_DOCK=False
  BACKWARD=False
)

ament_add_pytest_test(test_docking_server_with_non_charging_dock test_docking_server.py
  ENV
  NON_CHARGING_DOCK=True
  BACKWARD=False
)

ament_add_pytest_test(test_docking_server_backward test_docking_server.py
  ENV
  NON_CHARGING_DOCK=False
  BACKWARD=True
)

ament_add_pytest_test(test_docking_server_backward_blind test_docking_server.py
  ENV
  NON_CHARGING_DOCK=False
  BACKWARD_BLIND=True
)
