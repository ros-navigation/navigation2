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
target_link_libraries(${library_name} PUBLIC
  ${geographic_msgs_TARGETS}
  nav2_core::nav2_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  rclcpp_components::component
)

# Extract the major version (first number before the dot)
string(REGEX MATCH "^[0-9]+" ROS2_MAJOR_VERSION "${rclcpp_VERSION}")
# Convert to integer
if(ROS2_MAJOR_VERSION GREATER 29)
    message(STATUS "ROS2_MAJOR_VERSION (${ROS2_MAJOR_VERSION}) is greater than 29 (Kilted and newer).")
    target_link_libraries(${library_name} PUBLIC robot_localization::rl_lib)
else()
    message(STATUS "ROS2_MAJOR_VERSION (${ROS2_MAJOR_VERSION}) is NOT greater than 29 (Jazzy and older).")
    ament_target_dependencies(${library_name} PUBLIC robot_localization)
endif()

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${executable_name} PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

add_library(wait_at_waypoint SHARED plugins/wait_at_waypoint.cpp)
target_include_directories(wait_at_waypoint PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(wait_at_waypoint PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  nav2_core::nav2_core
)
target_link_libraries(wait_at_waypoint PRIVATE
  pluginlib::pluginlib
  nav2_util::nav2_util_core
)

add_library(photo_at_waypoint SHARED plugins/photo_at_waypoint.cpp)
target_include_directories(photo_at_waypoint PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(photo_at_waypoint PUBLIC
  cv_bridge::cv_bridge
  ${geometry_msgs_TARGETS}
  image_transport::image_transport
  nav2_core::nav2_core
  opencv_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
)
target_link_libraries(photo_at_waypoint PRIVATE
  nav2_util::nav2_util_core
  pluginlib::pluginlib
)

add_library(input_at_waypoint SHARED plugins/input_at_waypoint.cpp)
target_include_directories(input_at_waypoint PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(input_at_waypoint PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
)
target_link_libraries(input_at_waypoint PRIVATE
  pluginlib::pluginlib
  nav2_util::nav2_util_core
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
