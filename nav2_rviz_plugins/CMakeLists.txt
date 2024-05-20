cmake_minimum_required(VERSION 3.5)
project(nav2_rviz_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(nav2_rviz_plugins_headers_to_moc
  include/nav2_rviz_plugins/goal_pose_updater.hpp
  include/nav2_rviz_plugins/goal_common.hpp
  include/nav2_rviz_plugins/goal_tool.hpp
  include/nav2_rviz_plugins/nav2_panel.hpp
  include/nav2_rviz_plugins/particle_cloud_display/flat_weighted_arrows_array.hpp
  include/nav2_rviz_plugins/particle_cloud_display/particle_cloud_display.hpp
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/goal_tool.cpp
  src/nav2_panel.cpp
  src/particle_cloud_display/flat_weighted_arrows_array.cpp
  src/particle_cloud_display/particle_cloud_display.cpp
  ${nav2_rviz_plugins_headers_to_moc}
)
target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${library_name} PUBLIC
  rviz_common::rviz_common
  pluginlib::pluginlib
  ament_index_cpp::ament_index_cpp
  yaml-cpp
  ${geometry_msgs_TARGETS}
  nav2_util::nav2_util_core
  nav2_lifecycle_manager::nav2_lifecycle_manager_core
  rclcpp_action::rclcpp_action
  ${nav2_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
)

set(dependencies
  geometry_msgs
  nav2_util
  nav2_lifecycle_manager
  nav2_msgs
  nav_msgs
  pluginlib
  Qt5
  rclcpp
  rclcpp_lifecycle
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  std_msgs
  tf2_geometry_msgs
  yaml_cpp_vendor
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# TODO: Make this specific to this project (not rviz default plugins)
target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  map_msgs
  nav_msgs
  rclcpp
)

ament_package()
