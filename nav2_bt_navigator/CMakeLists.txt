cmake_minimum_required(VERSION 3.5)
project(nav2_bt_navigator)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

include_directories(
  include
)

set(bt_navigator_library_name bt_navigator)
set(navigate_to_pose_executable_name bt_navigator)
set(follow_waypoints_executable_name follow_waypoints_bt_navigator)

add_executable(${navigate_to_pose_executable_name}
  src/navigate_to_pose/main.cpp
)
add_executable(${follow_waypoints_executable_name}
  src/follow_waypoints/main.cpp
)

set(library_name ${bt_navigator_library_name}_core)

add_library(${library_name} SHARED
  src/bt_navigator.cpp
  src/navigate_to_pose/bt_navigator.cpp
  src/follow_waypoints/bt_navigator.cpp
  src/ros_topic_logger.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav_msgs
  nav2_msgs
  behaviortree_cpp_v3
  std_srvs
  nav2_util
  tf2_ros
)

ament_target_dependencies(${navigate_to_pose_executable_name}
  ${dependencies}
)
ament_target_dependencies(${follow_waypoints_executable_name}
  ${dependencies}
)

target_link_libraries(${navigate_to_pose_executable_name} ${library_name})
target_link_libraries(${follow_waypoints_executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${navigate_to_pose_executable_name} ${follow_waypoints_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
