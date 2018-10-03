cmake_minimum_required(VERSION 3.5)
project(nav2_costmap_world_model)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_tasks)
find_package(nav2_util)
find_package(nav2_libs_msgs)
find_package(nav2_world_model_msgs)
find_package(nav_msgs)

include_directories(
  include
)

set(executable_name costmap_world_model)

add_executable(${executable_name}
  src/costmap_world_model.cpp
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name}
  src/costmap_world_model.cpp
)

set(dependencies
  rclcpp
  std_msgs
  nav2_tasks
  nav2_util
  nav2_libs_msgs
  nav2_world_model_msgs
  tf2_ros
  tf2
  nav_msgs
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${executable_name} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
