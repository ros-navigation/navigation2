cmake_minimum_required(VERSION 3.5)
project(nav2_recoveries)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
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

set(library_name recoveries_server_core)
set(executable_name recoveries_server)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
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
add_library(nav2_spin_recovery SHARED
  plugins/spin.cpp
)

ament_target_dependencies(nav2_spin_recovery
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(nav2_spin_recovery PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(nav2_backup_recovery SHARED
  plugins/back_up.cpp
)

ament_target_dependencies(nav2_backup_recovery
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(nav2_backup_recovery PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(nav2_wait_recovery SHARED
  plugins/wait.cpp
)

ament_target_dependencies(nav2_wait_recovery
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(nav2_wait_recovery PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core recovery_plugin.xml)

# Library
add_library(${library_name}
  src/recovery_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Executable
add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)


install(TARGETS ${library_name}
                nav2_backup_recovery
                nav2_spin_recovery
                nav2_wait_recovery
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

install(FILES recovery_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name}
  nav2_backup_recovery
  nav2_spin_recovery
  nav2_wait_recovery
)
ament_export_dependencies(${dependencies})
ament_package()
