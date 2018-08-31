cmake_minimum_required(VERSION 3.5)
project(nav2_costmap_world_model)

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
find_package(nav2_util)
find_package(nav2_util_msgs)
find_package(nav2_world_model_msgs)

include_directories(
  include
)

add_executable(world_model
  src/world_model.cpp
  src/main.cpp
)

ament_target_dependencies(world_model
  rclcpp
  std_msgs
  nav2_util
  nav2_util_msgs
  nav2_world_model_msgs
  tf2_ros
  tf2
)

install(TARGETS
  world_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  # This forces cppcheck to consider all files in this project to be C++,
  # including the headers which end with .h, which cppcheck would normally
  # consider to be C instead.
  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()
endif()

ament_export_include_directories(include)

ament_package()
