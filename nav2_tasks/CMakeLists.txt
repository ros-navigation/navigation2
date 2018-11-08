cmake_minimum_required(VERSION 3.5)
project(nav2_tasks)

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
find_package(builtin_interfaces REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behavior_tree_core REQUIRED)

include_directories(
  include
)

set(library_name tasks)

add_library(${library_name}
  src/behavior_tree_engine.cpp
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_msgs
  behavior_tree_core
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_tasks test/test_tasks.cpp)
  ament_target_dependencies(test_tasks ${dependencies})
endif()

ament_export_include_directories(include)
#ament_export_dependencies(rosidl_default_runtime)

ament_package()
