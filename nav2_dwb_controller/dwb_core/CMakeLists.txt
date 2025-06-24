cmake_minimum_required(VERSION 3.5)
project(dwb_core)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_ros_common REQUIRED)

nav2_package()

add_library(dwb_core SHARED
  src/dwb_local_planner.cpp
  src/publisher.cpp
  src/illegal_trajectory_tracker.cpp
  src/trajectory_utils.cpp
)
target_include_directories(dwb_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
target_link_libraries(dwb_core PUBLIC
  ${builtin_interfaces_TARGETS}
  ${dwb_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  ${nav_2d_msgs_TARGETS}
  nav_2d_utils::conversions
  nav_2d_utils::tf_help
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)

install(TARGETS dwb_core
  EXPORT dwb_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(dwb_core)
ament_export_dependencies(
  builtin_interfaces
  dwb_msgs
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_2d_msgs
  nav_2d_utils
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2_ros
  visualization_msgs
  nav2_ros_common
)
ament_export_targets(dwb_core)

pluginlib_export_plugin_description_file(nav2_core local_planner_plugin.xml)

ament_package()
