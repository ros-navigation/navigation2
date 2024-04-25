cmake_minimum_required(VERSION 3.5)
project(nav2_collision_monitor)

### Dependencies ###

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

### Header ###

nav2_package()

### Libraries and executables ###

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_util
  nav2_costmap_2d
  nav2_msgs
  visualization_msgs
)

set(monitor_executable_name collision_monitor)
set(detector_executable_name collision_detector)
set(monitor_library_name ${monitor_executable_name}_core)
set(detector_library_name ${detector_executable_name}_core)

add_library(${monitor_library_name} SHARED
  src/collision_monitor_node.cpp
  src/polygon.cpp
  src/velocity_polygon.cpp
  src/circle.cpp
  src/source.cpp
  src/scan.cpp
  src/pointcloud.cpp
  src/polygon_source.cpp
  src/range.cpp
  src/kinematics.cpp
)
add_library(${detector_library_name} SHARED
  src/collision_detector_node.cpp
  src/polygon.cpp
  src/velocity_polygon.cpp
  src/circle.cpp
  src/source.cpp
  src/scan.cpp
  src/pointcloud.cpp
  src/polygon_source.cpp
  src/range.cpp
  src/kinematics.cpp
)

add_executable(${monitor_executable_name}
  src/collision_monitor_main.cpp
)
add_executable(${detector_executable_name}
  src/collision_detector_main.cpp
)

ament_target_dependencies(${monitor_library_name}
  ${dependencies}
)
ament_target_dependencies(${detector_library_name}
  ${dependencies}
)

target_link_libraries(${monitor_executable_name}
  ${monitor_library_name}
)
target_link_libraries(${detector_executable_name}
  ${detector_library_name}
)

ament_target_dependencies(${monitor_executable_name}
  ${dependencies}
)

ament_target_dependencies(${detector_executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${monitor_library_name} "nav2_collision_monitor::CollisionMonitor")

rclcpp_components_register_nodes(${detector_library_name} "nav2_collision_monitor::CollisionDetector")

### Install ###

install(TARGETS ${monitor_library_name} ${detector_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${monitor_executable_name} ${detector_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

### Testing ###

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

### Ament stuff ###

ament_export_include_directories(include)
ament_export_libraries(${monitor_library_name} ${detector_library_name})
ament_export_dependencies(${dependencies})

ament_package()
