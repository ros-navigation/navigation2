cmake_minimum_required(VERSION 3.5)
project(nav2_default_recoveries)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_recoveries REQUIRED)
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
  nav2_recoveries
  nav2_util
  nav2_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav2_core
  pluginlib
)

# plugins
add_library(nav2_default_recoveries SHARED
  src/spin.cpp
  src/back_up.cpp
  src/wait.cpp
)

ament_target_dependencies(nav2_default_recoveries
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(nav2_default_recoveries PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core recovery_plugins.xml)

install(TARGETS nav2_default_recoveries
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
ament_export_libraries(nav2_default_recoveries)
ament_package()
