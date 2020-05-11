cmake_minimum_required(VERSION 3.5)
project(nav2_recovery_plugins)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_recovery REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  nav2_recovery
  nav2_util
  nav2_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav2_core
  pluginlib
)

# plugins
add_library(nav2_recovery_plugins SHARED
  src/spin.cpp
  src/back_up.cpp
  src/wait.cpp
)

ament_target_dependencies(nav2_recovery_plugins
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(nav2_recovery_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core recovery_plugins.xml)

install(TARGETS nav2_recovery_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES recovery_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_recovery_plugins)
ament_package()
