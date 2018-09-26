cmake_minimum_required(VERSION 3.5)
project(nav2_dijkstra_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_tasks REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_planning_msgs REQUIRED)
find_package(nav2_libs_msgs REQUIRED)
find_package(nav2_world_model_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

set(executable_name dijkstra_planner)

add_executable(${executable_name}
  src/dijkstra_planner.cpp
  src/navfn.cpp
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name}
  src/dijkstra_planner.cpp
  src/navfn.cpp
)

set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
  nav2_tasks
  nav2_util
  nav2_planning_msgs
  nav2_libs_msgs
  nav2_world_model_msgs
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

ament_package()
