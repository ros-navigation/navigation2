cmake_minimum_required(VERSION 3.5)
project(nav2_waypoint_follower)

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name waypoint_follower)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/waypoint_follower.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  nav_msgs
  nav2_msgs
  nav2_util
  tf2_ros
  nav2_core
  pluginlib
  image_transport
  cv_bridge
  OpenCV
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(wait_at_waypoint SHARED plugins/wait_at_waypoint.cpp)
ament_target_dependencies(wait_at_waypoint ${dependencies})

add_library(photo_at_waypoint SHARED plugins/photo_at_waypoint.cpp)
ament_target_dependencies(photo_at_waypoint ${dependencies})

add_library(input_at_waypoint SHARED plugins/input_at_waypoint.cpp)
ament_target_dependencies(input_at_waypoint ${dependencies})

rclcpp_components_register_nodes(${library_name} "nav2_waypoint_follower::WaypointFollower")

install(TARGETS ${library_name} wait_at_waypoint photo_at_waypoint input_at_waypoint
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(wait_at_waypoint photo_at_waypoint input_at_waypoint ${library_name})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_waypoint_follower plugins.xml)

ament_package()
