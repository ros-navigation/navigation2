cmake_minimum_required(VERSION 3.5)
project(nav2_motion_primitives)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_tasks REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_robot REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

set(library_name motion_primitives)
set(executable_name ${library_name}_node)

set(dependencies
  rclcpp
  std_msgs
  nav2_util
  nav2_tasks
  nav2_msgs
  nav2_robot
  nav_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

# Library
add_library(${library_name}
  src/spin.cpp
  src/back_up.cpp
  src/stop.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# Executable
add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

install(TARGETS ${executable_name} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})

ament_package()
