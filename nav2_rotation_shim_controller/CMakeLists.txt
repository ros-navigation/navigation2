cmake_minimum_required(VERSION 3.5)
project(nav2_rotation_shim_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_ros_common REQUIRED)

nav2_package()

set(library_name nav2_rotation_shim_controller)

add_library(${library_name} SHARED
  src/nav2_rotation_shim_controller.cpp)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_controller::position_goal_checker
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  angles::angles
  nav2_util::nav2_util_core
  tf2::tf2
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

  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_controller
  nav2_costmap_2d
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  tf2_ros
  nav2_ros_common
)
ament_export_targets(${library_name})

pluginlib_export_plugin_description_file(nav2_core nav2_rotation_shim_controller.xml)

ament_package()
