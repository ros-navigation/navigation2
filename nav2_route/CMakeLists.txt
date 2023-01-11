cmake_minimum_required(VERSION 3.5)
project(nav2_route)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)

nav2_package()

set(executable_name route_server)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  visualization_msgs
  nav_msgs
  tf2_ros
)

include_directories(
  include
)


add_library(${library_name} SHARED
  src/route_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "nav2_route::RouteServer")

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${library_name})
ament_package()
















