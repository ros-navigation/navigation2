cmake_minimum_required(VERSION 3.5)
project(nav2_semantic_map_server)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

nav2_package()

set(semantic_map_server_executable semantic_map_server)

add_executable(${semantic_map_server_executable}
  src/main.cpp)

set(dependencies
  rclcpp
  nav_msgs
  nav2_msgs)

ament_target_dependencies(${semantic_map_server_executable}
  ${dependencies})

install(TARGETS
  ${semantic_map_server_executable}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
