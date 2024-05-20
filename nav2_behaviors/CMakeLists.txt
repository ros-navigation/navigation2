cmake_minimum_required(VERSION 3.5)
project(nav2_behaviors)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

set(library_name behavior_server_core)
set(executable_name behavior_server)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  nav2_util
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav2_costmap_2d
  nav2_core
  pluginlib
)

# plugins
add_library(nav2_spin_behavior SHARED
  plugins/spin.cpp
)
target_include_directories(nav2_spin_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav2_spin_behavior PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  tf2::tf2
  nav2_util::nav2_util_core
  nav2_costmap_2d::nav2_costmap_2d_core
)
target_link_libraries(nav2_spin_behavior PRIVATE
  ${tf2_geometry_msgs_TARGETS}
  pluginlib::pluginlib
)

add_library(nav2_wait_behavior SHARED
  plugins/wait.cpp
)
target_include_directories(nav2_wait_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav2_wait_behavior PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  tf2::tf2
  nav2_util::nav2_util_core
  nav2_costmap_2d::nav2_costmap_2d_core
)

add_library(nav2_drive_on_heading_behavior SHARED
  plugins/drive_on_heading.cpp
)
target_include_directories(nav2_drive_on_heading_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav2_drive_on_heading_behavior PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  tf2::tf2
  nav2_util::nav2_util_core
  nav2_costmap_2d::nav2_costmap_2d_core
)
target_link_libraries(nav2_drive_on_heading_behavior PRIVATE
  pluginlib::pluginlib
)

add_library(nav2_back_up_behavior SHARED
  plugins/back_up.cpp
)
target_include_directories(nav2_back_up_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav2_back_up_behavior PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  tf2::tf2
  nav2_util::nav2_util_core
  nav2_costmap_2d::nav2_costmap_2d_core
)
target_link_libraries(nav2_back_up_behavior PRIVATE
  pluginlib::pluginlib
)

add_library(nav2_assisted_teleop_behavior SHARED
  plugins/assisted_teleop.cpp
)
target_include_directories(nav2_assisted_teleop_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav2_assisted_teleop_behavior PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  nav2_core::nav2_core
  tf2::tf2
  nav2_util::nav2_util_core
  nav2_costmap_2d::nav2_costmap_2d_core
)
target_link_libraries(nav2_assisted_teleop_behavior PRIVATE
  pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

# Library
add_library(${library_name} SHARED
  src/behavior_server.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  rclcpp_lifecycle::rclcpp_lifecycle
  nav2_util::nav2_util_core
  tf2_ros::tf2_ros
  pluginlib::pluginlib
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_costmap_2d::nav2_costmap_2d_client
)
target_link_libraries(${library_name} PRIVATE
  rclcpp_components::component
)

# Executable
add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(${executable_name} PRIVATE rclcpp::rclcpp ${library_name})

rclcpp_components_register_nodes(${library_name} "behavior_server::BehaviorServer")

install(TARGETS nav2_spin_behavior
                nav2_wait_behavior
                nav2_assisted_teleop_behavior
                nav2_drive_on_heading_behavior
                nav2_back_up_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${library_name}
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

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/plugins/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp_action REQUIRED)

  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name}
  nav2_spin_behavior
  nav2_wait_behavior
  nav2_assisted_teleop_behavior
  nav2_drive_on_heading_behavior
  nav2_back_up_behavior
)
ament_export_dependencies(${dependencies})
ament_export_targets(${library_name})
ament_package()
