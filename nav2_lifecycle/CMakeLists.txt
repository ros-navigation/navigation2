cmake_minimum_required(VERSION 3.5)
project(nav2_lifecycle)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_common REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_node.cpp
  src/lifecycle_service_client.cpp
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  nav2_common
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS
  ${library_name}
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
ament_export_dependencies(${dependencies})

ament_package()
