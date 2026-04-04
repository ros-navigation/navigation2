cmake_minimum_required(VERSION 3.8)
project(nav2_loopback_sim)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(executable_name loopback_simulator)
set(library_name ${executable_name}_core)

# Main library
add_library(${library_name} SHARED
  src/loopback_simulator.cpp
  src/clock_publisher.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  nav2_ros_common::nav2_ros_common
  nav2_util::nav2_util_core
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rosgraph_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  rclcpp_components::component
)

# Main executable
add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(${executable_name} PRIVATE ${library_name} rclcpp::rclcpp)

rclcpp_components_register_nodes(${library_name} "nav2_loopback_sim::LoopbackSimulator")
rclcpp_components_register_nodes(${library_name} "nav2_loopback_sim::ClockPublisher")

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Clock publisher executable
set(clock_publisher_executable clock_publisher)
add_executable(${clock_publisher_executable}
  src/clock_publisher_main.cpp
)
target_link_libraries(${clock_publisher_executable} PRIVATE ${library_name} rclcpp::rclcpp)

install(TARGETS ${executable_name} ${clock_publisher_executable}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_ros_common
  rcl_interfaces
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  rosgraph_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
ament_package()
