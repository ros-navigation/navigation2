cmake_minimum_required(VERSION 3.5)
project(nav2_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_lifecycle REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/nav2_controller.cpp
  src/nav2_controller_client.cpp
)

set(dependencies
  std_msgs
  lifecycle_msgs
  rclcpp_lifecycle
  rclcpp
  diagnostic_msgs
  geometry_msgs
  std_srvs
  nav2_msgs
  nav2_lifecycle
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(nav2_controller
  src/main.cpp
)

target_link_libraries(nav2_controller
  ${library_name}
)

ament_target_dependencies(nav2_controller
  ${dependencies}
)

add_executable(nav2_cli
  src/cli/nav2_cli.cpp
)

target_link_libraries(nav2_cli
  ${library_name}
)

ament_target_dependencies(nav2_cli
  ${dependencies}
)

install(TARGETS
  nav2_controller nav2_cli
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY src/gui DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
