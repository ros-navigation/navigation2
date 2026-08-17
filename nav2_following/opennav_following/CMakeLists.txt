cmake_minimum_required(VERSION 3.5)
project(opennav_following)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(opennav_docking REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(executable_name opennav_following)
set(library_name ${executable_name}_core)

set(dependencies
  angles
  geometry_msgs
  nav2_msgs
  nav2_util
  opennav_docking
  opennav_docking_core
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(${library_name} SHARED
  src/parameter_handler.cpp
  src/following_server.cpp
)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "opennav_following::FollowingServer")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_lint_auto_find_test_dependencies()
  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_msgs
  nav2_util
  opennav_docking
  opennav_docking_core
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)
ament_package()
