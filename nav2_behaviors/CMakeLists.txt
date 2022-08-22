cmake_minimum_required(VERSION 3.5)
project(nav2_behaviors)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name behavior_server_core)
set(executable_name behavior_server)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  nav2_util
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  tf2
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

ament_target_dependencies(nav2_spin_behavior
  ${dependencies}
)

add_library(nav2_wait_behavior SHARED
plugins/wait.cpp
)

ament_target_dependencies(nav2_wait_behavior
${dependencies}
)

add_library(nav2_drive_on_heading_behavior SHARED
  plugins/drive_on_heading.cpp
)

ament_target_dependencies(nav2_drive_on_heading_behavior
  ${dependencies}
)

add_library(nav2_back_up_behavior SHARED
  plugins/back_up.cpp
)

ament_target_dependencies(nav2_back_up_behavior
  ${dependencies}
)

add_library(nav2_assisted_teleop_behavior SHARED
  plugins/assisted_teleop.cpp
)

ament_target_dependencies(nav2_assisted_teleop_behavior
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

# Library
add_library(${library_name} SHARED
  src/behavior_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# Executable
add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "behavior_server::BehaviorServer")

install(TARGETS ${library_name}
                nav2_spin_behavior
                nav2_wait_behavior
                nav2_assisted_teleop_behavior
                nav2_drive_on_heading_behavior
                nav2_back_up_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
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
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name}
  nav2_spin_behavior
  nav2_wait_behavior
  nav2_assisted_teleop_behavior
  nav2_drive_on_heading_behavior
  nav2_back_up_behavior
)
ament_export_dependencies(${dependencies})
ament_package()
