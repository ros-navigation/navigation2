cmake_minimum_required(VERSION 3.5)
project(nav2_dwa_controller)

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
find_package(nav2_tasks REQUIRED)
find_package(nav2_robot REQUIRED)
find_package(nav2_planning_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dwb_local_planner REQUIRED)
find_package(dwb_plugins REQUIRED)
find_package(dwb_critics REQUIRED)

include_directories(
  include
)

set(executable_name dwa_controller)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name}
  src/dwa_controller.cpp
)

set(dependencies
  rclcpp
  std_msgs
  nav2_tasks
  nav2_robot
  nav2_planning_msgs
  geometry_msgs
  dwb_local_planner
  dwb_plugins
  dwb_critics
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name} ${library_name})

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

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_dependencies(dwb_msgs)

ament_package()
