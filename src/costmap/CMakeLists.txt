cmake_minimum_required(VERSION 3.5)
project(costmap)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_library(costmap STATIC
  src/Costmap.cpp
)

ament_target_dependencies(costmap
  nav2_msgs
  tf2_ros
  tf2
)

install(TARGETS
  costmap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_package()
