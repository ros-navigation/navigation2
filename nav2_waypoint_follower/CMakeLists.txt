cmake_minimum_required(VERSION 3.5)
project(nav2_waypoint_follower)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_localization REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(executable_name waypoint_follower)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/waypoint_follower.cpp
)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name}
  geographic_msgs
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2_ros
  rclcpp_components
)

# Extract the major version (first number before the dot)
string(REGEX MATCH "^[0-9]+" ROS2_MAJOR_VERSION "${rclcpp_VERSION}")
# Convert to integer
if(ROS2_MAJOR_VERSION GREATER_EQUAL 29)
    message(STATUS "ROS2_MAJOR_VERSION (${ROS2_MAJOR_VERSION}) is greater than 29 (Kilted and newer).")
    target_link_libraries(${library_name} robot_localization::rl_lib)
else()
    message(STATUS "ROS2_MAJOR_VERSION (${ROS2_MAJOR_VERSION}) is NOT greater than 29 (Jazzy and older).")
    ament_target_dependencies(${library_name} robot_localization)
endif()

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${executable_name}
  rclcpp
)
target_link_libraries(${executable_name}
  ${library_name}
)

add_library(wait_at_waypoint SHARED plugins/wait_at_waypoint.cpp)
target_include_directories(wait_at_waypoint PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(wait_at_waypoint
  geometry_msgs
  rclcpp
  rclcpp_lifecycle
  nav2_core
  pluginlib
  nav2_util
)

add_library(photo_at_waypoint SHARED plugins/photo_at_waypoint.cpp)
target_include_directories(photo_at_waypoint PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(photo_at_waypoint
  geometry_msgs
  image_transport
  nav2_core
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  nav2_util
  pluginlib
  cv_bridge
)
target_link_libraries(photo_at_waypoint
  opencv_core
)

add_library(input_at_waypoint SHARED plugins/input_at_waypoint.cpp)
target_include_directories(input_at_waypoint PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(input_at_waypoint
  geometry_msgs
  nav2_core
  rclcpp
  rclcpp_lifecycle
  std_msgs
  pluginlib
  nav2_util
)

rclcpp_components_register_nodes(${library_name} "nav2_waypoint_follower::WaypointFollower")

install(TARGETS ${library_name} wait_at_waypoint photo_at_waypoint input_at_waypoint
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(wait_at_waypoint photo_at_waypoint input_at_waypoint ${library_name})
ament_export_dependencies(
  cv_bridge
  geographic_msgs
  geometry_msgs
  image_transport
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  OpenCV
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  robot_localization
  sensor_msgs
  std_msgs
  tf2_ros
)
ament_export_targets(${PROJECT_NAME})
pluginlib_export_plugin_description_file(nav2_waypoint_follower plugins.xml)

ament_package()
