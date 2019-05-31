cmake_minimum_required(VERSION 3.5)
project(nav2_robot)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)

nav2_package()

include_directories(
  include
)

add_library(nav2_robot SHARED
  src/robot.cpp
)

ament_target_dependencies(nav2_robot
  rclcpp
  rclcpp_lifecycle
  urdf
  geometry_msgs
  nav_msgs
  nav2_util
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(urdf)
ament_export_dependencies(nav_msgs)

ament_package()
