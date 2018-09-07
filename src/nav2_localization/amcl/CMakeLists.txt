cmake_minimum_required(VERSION 3.5)
project(amcl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(Boost REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)

include_directories(
  include 
  ${Boost_INCLUDE_DIRS}
  )

add_executable(amcl src/main.cpp src/amcl_node.cpp)

target_link_libraries(amcl
  ${Boost_LIBRARIES}
  boost_system
  )

ament_target_dependencies(amcl
  rclcpp
  message_filters
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  tf2
  nav2_util
  )

install(TARGETS
  amcl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
ament_export_include_directories(include)
ament_export_libraries(amcl)
ament_package()
