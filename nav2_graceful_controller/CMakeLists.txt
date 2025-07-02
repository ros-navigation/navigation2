cmake_minimum_required(VERSION 3.5)
project(nav2_graceful_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_ros_common REQUIRED)

nav2_package()

# Add Smooth Control Law as library
add_library(smooth_control_law SHARED src/smooth_control_law.cpp)
target_include_directories(smooth_control_law
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
target_link_libraries(smooth_control_law PUBLIC
  angles::angles
  ${geometry_msgs_TARGETS}
  nav2_util::nav2_util_core
  tf2::tf2
)

# Add Graceful Controller
set(library_name nav2_graceful_controller)

add_library(${library_name} SHARED
  src/graceful_controller.cpp
  src/parameter_handler.cpp
  src/path_handler.cpp
  src/utils.cpp
  src/path_hug.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_util::nav2_util_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav_2d_utils::tf_help
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  smooth_control_law
  tf2::tf2
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${library_name} PRIVATE
  tf2_geometry_msgs::tf2_geometry_msgs
)

install(TARGETS smooth_control_law ${library_name}
  EXPORT nav2_graceful_controller
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(smooth_control_law ${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_util
  nav2_costmap_2d
  nav_2d_utils
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  tf2
  tf2_ros
  visualization_msgs
  nav2_ros_common
)
ament_export_targets(nav2_graceful_controller)

pluginlib_export_plugin_description_file(nav2_core graceful_controller_plugin.xml)

ament_package()
