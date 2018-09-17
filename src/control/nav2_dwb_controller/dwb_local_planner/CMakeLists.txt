cmake_minimum_required(VERSION 3.5)
project(dwb_local_planner)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)

include_directories(
  include
)

set(PROJECT_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_2d_msgs
  dwb_msgs
  costmap_2d
  pluginlib
  sensor_msgs
  visualization_msgs
  nav_2d_utils
)

add_library(dwb_local_planner
  src/dwb_local_planner.cpp
  src/publisher.cpp
  src/illegal_trajectory_tracker.cpp
)
ament_target_dependencies(dwb_local_planner
  ${PROJECT_DEPENDENCIES}
)

install(TARGETS dwb_local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

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

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  #add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(dwb_local_planner)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

ament_package()
