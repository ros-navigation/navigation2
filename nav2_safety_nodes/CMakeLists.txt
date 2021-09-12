cmake_minimum_required(VERSION 3.8)
project(nav2_safety_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

nav2_package()

link_libraries(stdc++fs)

include_directories(
  include
)

add_library(nav2_safety_node_core SHARED
  src/nav2_safety_node.cpp
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  laser_geometry
  nav2_util
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

ament_target_dependencies(nav2_safety_node_core
  ${dependencies}
)

add_executable(nav2_safety_node
  src/main.cpp
)

target_link_libraries(nav2_safety_node
  nav2_safety_node_core
)

ament_target_dependencies(nav2_safety_node
  ${dependencies}
)

install(TARGETS
nav2_safety_node_core 
RUNTIME DESTINATION lib
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib)

install(TARGETS
nav2_safety_node 
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_detect_testING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_detect_test_dependencies()

endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_safety_node_core)
ament_export_dependencies(${dependencies})
ament_package()