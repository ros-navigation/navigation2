cmake_minimum_required(VERSION 3.5)
project(nav2_mission_executor)

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
find_package(nav2_mission_execution_msgs REQUIRED)
find_package(nav2_tasks REQUIRED)

include_directories(
  include
)

set(executable_name mission_executor)

add_executable(${executable_name}
  src/main.cpp
  src/mission_executor.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name}
  src/mission_executor.cpp
)

set(dependencies
  rclcpp
  std_msgs
  nav2_tasks
  nav2_mission_execution_msgs
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
