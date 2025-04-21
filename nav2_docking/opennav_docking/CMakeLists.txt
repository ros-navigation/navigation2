cmake_minimum_required(VERSION 3.5)
project(opennav_docking)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_graceful_controller REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

nav2_package()

set(executable_name opennav_docking)
set(library_name ${executable_name}_core)

add_library(controller SHARED
  src/controller.cpp
)
target_include_directories(controller
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(controller PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_graceful_controller::nav2_graceful_controller
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
)

add_library(${library_name} SHARED
  src/docking_server.cpp
  src/dock_database.cpp
  src/navigator.cpp
)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  angles::angles
  controller
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  opennav_docking_core::opennav_docking_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2_ros::tf2_ros
  yaml-cpp::yaml-cpp
)
target_link_libraries(${library_name} PRIVATE
  rclcpp_components::component
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_library(pose_filter SHARED
  src/pose_filter.cpp
)
target_include_directories(pose_filter
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(pose_filter PUBLIC
  ${geometry_msgs_TARGETS}
)
target_link_libraries(pose_filter PRIVATE
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${executable_name} PRIVATE ${library_name} rclcpp::rclcpp)

rclcpp_components_register_nodes(${library_name} "opennav_docking::DockingServer")

add_library(simple_charging_dock SHARED
  src/simple_charging_dock.cpp
)
target_include_directories(simple_charging_dock
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(simple_charging_dock PUBLIC
  ${geometry_msgs_TARGETS}
  opennav_docking_core::opennav_docking_core
  pose_filter
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)
target_link_libraries(simple_charging_dock PRIVATE
  angles::angles
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  tf2::tf2
)

add_library(simple_non_charging_dock SHARED
  src/simple_non_charging_dock.cpp
)
target_include_directories(simple_non_charging_dock
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(simple_non_charging_dock PUBLIC
  ${geometry_msgs_TARGETS}
  opennav_docking_core::opennav_docking_core
  pose_filter
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)
target_link_libraries(simple_non_charging_dock PRIVATE
  angles::angles
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  tf2::tf2
)

pluginlib_export_plugin_description_file(opennav_docking_core plugins.xml)

install(TARGETS ${library_name} controller pose_filter simple_charging_dock simple_non_charging_dock
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

install(DIRECTORY test/dock_files
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  ament_lint_auto_find_test_dependencies()
  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name} controller pose_filter)
ament_export_dependencies(
  geometry_msgs
  nav2_graceful_controller
  nav2_msgs
  nav2_util
  opennav_docking_core
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rcl_interfaces
  tf2_ros
  yaml-cpp
)
ament_export_targets(${PROJECT_NAME})
ament_package()
