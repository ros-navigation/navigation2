cmake_minimum_required(VERSION 3.5)
project(nav2_system_tests)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
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
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(navigation2)
find_package(angles REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

nav2_package()

set(dependencies
  rclcpp
  nav2_util
  nav2_map_server
  nav2_msgs
  nav_msgs
  visualization_msgs
  nav2_amcl
  nav2_lifecycle_manager
  nav2_behavior_tree
  gazebo_ros_pkgs
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  rclpy
  nav2_planner
  nav2_navfn_planner
  angles
  behaviortree_cpp_v3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  add_subdirectory(src/behavior_tree)
  add_subdirectory(src/planning)
  add_subdirectory(src/localization)
  add_subdirectory(src/system)
  add_subdirectory(src/system_failure)
  add_subdirectory(src/updown)
  add_subdirectory(src/waypoint_follower)
  add_subdirectory(src/recoveries/spin)
  add_subdirectory(src/recoveries/wait)
  add_subdirectory(src/recoveries/backup)
  add_subdirectory(src/costmap_filters)
  install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})

endif()

ament_package()
