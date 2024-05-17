cmake_minimum_required(VERSION 3.5)
project(nav2_util)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(bond REQUIRED)
find_package(rcl_interfaces REQUIRED)

nav2_package()

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/costmap.cpp
  src/node_utils.cpp
  src/lifecycle_service_client.cpp
  src/string_utils.cpp
  src/lifecycle_utils.cpp
  src/lifecycle_node.cpp
  src/robot_utils.cpp
  src/node_thread.cpp
  src/odometry_utils.cpp
)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${library_name} PUBLIC
  ${nav2_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  ${lifecycle_msgs_TARGETS}
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  bondcpp::bondcpp
  ${bond_TARGETS}
  ${rcl_interfaces_TARGETS}
)

add_executable(lifecycle_bringup
  src/lifecycle_bringup_commandline.cpp
)
target_link_libraries(lifecycle_bringup PRIVATE ${library_name} rclcpp::rclcpp)

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_bringup
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  nav2_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  lifecycle_msgs
  rclcpp_action
  rclcpp_lifecycle
  bondcpp
  bond
  rcl_interfaces
)
ament_export_targets(${library_name})

ament_package()
