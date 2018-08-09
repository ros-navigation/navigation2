cmake_minimum_required(VERSION 3.5)
project(world_model)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(costmap)

include_directories(
  include
  ../costmap/include
)

# TODO(orduno): CMAKE is searching /usr/bin/ld for the library
# link_directories(${CMAKE_BINARY_DIR}/../libs)
link_directories(~/ros2_overlay_ws/build/costmap/)

add_executable(world_model
  src/WorldModel.cpp
  src/main.cpp
)

target_link_libraries(
  world_model
  costmap
)

ament_target_dependencies(world_model
  rclcpp
  std_msgs
  nav2_msgs
  tf2_ros
  tf2
  costmap
)

install(TARGETS
  world_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
