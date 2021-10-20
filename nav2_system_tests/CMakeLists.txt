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

  # add_subdirectory(src/behavior_tree) # OK
  # add_subdirectory(src/planning) # OK
  # add_subdirectory(src/localization) # OK
  # add_subdirectory(src/system) # OK
  # add_subdirectory(src/system_failure) # OK
  # add_subdirectory(src/updown) # OK
  # add_subdirectory(src/waypoint_follower) # OK
  # add_subdirectory(src/recoveries/spin) # OK
  # add_subdirectory(src/recoveries/wait) # OK
  # add_subdirectory(src/recoveries/backup) # FAIL
  # add_subdirectory(src/costmap_filters) # FAIL
  install(DIRECTORY maps models worlds DESTINATION share/${PROJECT_NAME})

  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/nav2_system_tests.dsv.in")

endif()

ament_package()
