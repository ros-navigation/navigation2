cmake_minimum_required(VERSION 3.5)
project(nav2_lifecycle_manager)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)

set(dependencies
  geometry_msgs
  lifecycle_msgs
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  std_srvs
  tf2_geometry_msgs
  bondcpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(lifecycle_manager
  src/main.cpp
)

target_link_libraries(lifecycle_manager
  ${library_name}
)

ament_target_dependencies(lifecycle_manager
  ${dependencies}
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
