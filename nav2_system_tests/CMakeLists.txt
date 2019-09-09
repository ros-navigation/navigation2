cmake_minimum_required(VERSION 3.5)
project(nav2_system_tests)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(rclpy REQUIRED)
find_package(navigation2)

nav2_package()

set(dependencies
  rclcpp
  nav2_util
  nav2_msgs
  nav_msgs
  visualization_msgs
  nav2_amcl
  nav2_lifecycle_manager
  gazebo_ros_pkgs
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  rclpy
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  add_subdirectory(src/planning)
  add_subdirectory(src/localization)
  add_subdirectory(src/system)
  add_subdirectory(src/updown)

endif()

ament_package()
