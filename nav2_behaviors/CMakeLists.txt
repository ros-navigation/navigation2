cmake_minimum_required(VERSION 3.5)
project(nav2_behaviors)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(dependencies
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  pluginlib
  tf2_geometry_msgs
)

set(library_name behavior_server_core)
set(executable_name behavior_server)

# plugins
add_library(nav2_spin_behavior SHARED
  plugins/spin.cpp
)
target_include_directories(nav2_spin_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(nav2_spin_behavior
  ${dependencies}
)
# target_link_libraries(nav2_spin_behavior PUBLIC
#   ${geometry_msgs_TARGETS}
#   nav2_core::nav2_core
#   nav2_costmap_2d::nav2_costmap_2d_client
#   nav2_costmap_2d::nav2_costmap_2d_core
#   ${nav2_msgs_TARGETS}
#   nav2_util::nav2_util_core
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   tf2::tf2
#   tf2_ros::tf2_ros
# )
# target_link_libraries(nav2_spin_behavior PRIVATE
#   pluginlib::pluginlib
#   tf2_geometry_msgs::tf2_geometry_msgs
# )

add_library(nav2_wait_behavior SHARED
  plugins/wait.cpp
)
target_include_directories(nav2_wait_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(nav2_wait_behavior
  ${dependencies}
)
# target_link_libraries(nav2_wait_behavior PUBLIC
#   ${geometry_msgs_TARGETS}
#   nav2_core::nav2_core
#   nav2_costmap_2d::nav2_costmap_2d_core
#   ${nav2_msgs_TARGETS}
#   nav2_util::nav2_util_core
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   tf2::tf2
#   tf2_ros::tf2_ros
# )

add_library(nav2_drive_on_heading_behavior SHARED
  plugins/drive_on_heading.cpp
)
target_include_directories(nav2_drive_on_heading_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(nav2_drive_on_heading_behavior
  ${dependencies}
)
# target_link_libraries(nav2_drive_on_heading_behavior PUBLIC
#   ${geometry_msgs_TARGETS}
#   nav2_core::nav2_core
#   nav2_costmap_2d::nav2_costmap_2d_core
#   ${nav2_msgs_TARGETS}
#   nav2_util::nav2_util_core
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   tf2::tf2
#   tf2_ros::tf2_ros
# )
# target_link_libraries(nav2_drive_on_heading_behavior PRIVATE
#   pluginlib::pluginlib
# )

add_library(nav2_back_up_behavior SHARED
  plugins/back_up.cpp
)
target_include_directories(nav2_back_up_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(nav2_back_up_behavior
  ${dependencies}
)
# target_link_libraries(nav2_back_up_behavior PUBLIC
#   ${geometry_msgs_TARGETS}
#   nav2_core::nav2_core
#   nav2_costmap_2d::nav2_costmap_2d_core
#   ${nav2_msgs_TARGETS}
#   nav2_util::nav2_util_core
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   tf2::tf2
#   tf2_ros::tf2_ros
# )
# target_link_libraries(nav2_back_up_behavior PRIVATE
#   pluginlib::pluginlib
# )

add_library(nav2_assisted_teleop_behavior SHARED
  plugins/assisted_teleop.cpp
)
target_include_directories(nav2_assisted_teleop_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(nav2_assisted_teleop_behavior
  ${dependencies}
)
# target_link_libraries(nav2_assisted_teleop_behavior PUBLIC
#   ${geometry_msgs_TARGETS}
#   nav2_core::nav2_core
#   nav2_costmap_2d::nav2_costmap_2d_core
#   ${nav2_msgs_TARGETS}
#   nav2_util::nav2_util_core
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   ${std_msgs_TARGETS}
#   tf2::tf2
#   tf2_ros::tf2_ros
# )
# target_link_libraries(nav2_assisted_teleop_behavior PRIVATE
#   pluginlib::pluginlib
# )

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
ament_target_dependencies(${library_name}
  ${dependencies}
  rclcpp_components
)
# target_link_libraries(${library_name} PUBLIC
#   nav2_core::nav2_core
#   nav2_costmap_2d::nav2_costmap_2d_client
#   nav2_costmap_2d::nav2_costmap_2d_core
#   nav2_util::nav2_util_core
#   pluginlib::pluginlib
#   rclcpp_lifecycle::rclcpp_lifecycle
#   tf2_ros::tf2_ros
# )
# target_link_libraries(${library_name} PRIVATE
#   rclcpp_components::component
# )

# Executable
add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(${executable_name} PRIVATE rclcpp::rclcpp ${library_name})

rclcpp_components_register_nodes(${library_name} "behavior_server::BehaviorServer")

install(
  TARGETS
    ${library_name}
    nav2_spin_behavior
    nav2_wait_behavior
    nav2_assisted_teleop_behavior
    nav2_drive_on_heading_behavior
    nav2_back_up_behavior
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
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_ros
)
ament_export_targets(${library_name})
ament_package()
