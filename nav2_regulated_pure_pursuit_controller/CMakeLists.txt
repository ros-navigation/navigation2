cmake_minimum_required(VERSION 3.8)
project(nav2_regulated_pure_pursuit_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_ros_common REQUIRED)

nav2_package()

set(library_name nav2_regulated_pure_pursuit_controller)

add_library(${library_name} SHARED
  src/regulated_pure_pursuit_controller.cpp
  src/collision_checker.cpp
  src/parameter_handler.cpp)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_ros_common::nav2_ros_common
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  angles::angles
)

install(TARGETS ${library_name}
  EXPORT ${library_name}
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

  find_package(tf2_geometry_msgs REQUIRED)

  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  nav2_ros_common
  std_msgs
  tf2
  tf2_ros
)
ament_export_targets(${library_name})

pluginlib_export_plugin_description_file(nav2_core nav2_regulated_pure_pursuit_controller.xml)

ament_package()
