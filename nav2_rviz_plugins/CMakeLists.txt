cmake_minimum_required(VERSION 3.5)
project(nav2_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_route REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

nav2_package()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.
set(nav2_rviz_plugins_headers_to_moc
  include/nav2_rviz_plugins/costmap_cost_tool.hpp
  include/nav2_rviz_plugins/docking_panel.hpp
  include/nav2_rviz_plugins/goal_pose_updater.hpp
  include/nav2_rviz_plugins/goal_tool.hpp
  include/nav2_rviz_plugins/nav2_panel.hpp
  include/nav2_rviz_plugins/route_tool.hpp
  include/nav2_rviz_plugins/selector.hpp
  include/nav2_rviz_plugins/particle_cloud_display/particle_cloud_display.hpp
)

foreach(header "${nav2_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(nav2_rviz_plugins_moc_files "${header}")
endforeach()

qt5_wrap_ui(route_tool_UIS_H resource/route_tool.ui)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/costmap_cost_tool.cpp
  src/docking_panel.cpp
  src/goal_tool.cpp
  src/nav2_panel.cpp
  src/route_tool.cpp
  src/selector.cpp
  src/utils.cpp
  src/particle_cloud_display/flat_weighted_arrows_array.cpp
  src/particle_cloud_display/particle_cloud_display.cpp
  ${nav2_rviz_plugins_moc_files}
  ${route_tool_UIS_H}
)
target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_lifecycle_manager::nav2_lifecycle_manager_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  nav2_route::route_server_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rviz_common::rviz_common
  rviz_default_plugins::rviz_default_plugins
  rviz_rendering::rviz_rendering
  ${std_msgs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${library_name} PRIVATE
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  yaml-cpp
)

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

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

install(DIRECTORY rviz launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  nav2_lifecycle_manager
  nav2_msgs
  nav2_route
  nav2_util
  Qt5
  rclcpp
  rclcpp_action
  rviz_common
  rviz_default_plugins
  rviz_rendering
  std_msgs
  tf2_geometry_msgs
  visualization_msgs
)

ament_package()
