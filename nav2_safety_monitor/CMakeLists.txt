cmake_minimum_required(VERSION 3.5)
project(nav2_safety_monitor)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

nav2_package()

include_directories(
  include
)

add_library(nav2_safety_monitor SHARED
  src/nav2_safety_monitor.cpp
  src/sensors/laser.cpp
  src/sensors/sonar.cpp
  src/sensors/contact.cpp
)

ament_target_dependencies(nav2_safety_monitor
  rclcpp
  sensor_msgs
  tf2_ros
  laser_geometry
  std_msgs
  geometry_msgs
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
