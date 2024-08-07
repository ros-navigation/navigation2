cmake_minimum_required(VERSION 3.5)
project(nav2_smoother)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(executable_name smoother_server)
set(library_name ${executable_name}_core)

# Main library
add_library(${library_name} SHARED
  src/nav2_smoother.cpp
)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${library_name} PUBLIC
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  nav_2d_utils::conversions
  nav_2d_utils::tf_help
  rclcpp_components::component
  tf2::tf2
)

# Main executable
add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${executable_name} PRIVATE rclcpp::rclcpp ${library_name})

# Simple Smoother plugin
add_library(simple_smoother SHARED
  src/simple_smoother.cpp
)
target_include_directories(simple_smoother
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(simple_smoother PUBLIC
  angles::angles
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_ros::tf2_ros
)
target_link_libraries(simple_smoother PRIVATE
  pluginlib::pluginlib
)

# Savitzky Golay Smoother plugin
add_library(savitzky_golay_smoother SHARED
  src/savitzky_golay_smoother.cpp
)
target_include_directories(savitzky_golay_smoother
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(savitzky_golay_smoother PUBLIC
  angles::angles
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_ros::tf2_ros
)
target_link_libraries(savitzky_golay_smoother PRIVATE
  pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(rclcpp_action REQUIRED)

  ament_find_gtest()
  add_subdirectory(test)
endif()

rclcpp_components_register_nodes(${library_name} "nav2_smoother::SmootherServer")

install(
  TARGETS ${library_name} simple_smoother savitzky_golay_smoother
  EXPORT ${library_name}
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

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name} simple_smoother savitzky_golay_smoother)
ament_export_dependencies(
  angles
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
)
ament_export_targets(${library_name})
ament_package()
