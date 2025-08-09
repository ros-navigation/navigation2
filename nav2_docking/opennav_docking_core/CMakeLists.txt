cmake_minimum_required(VERSION 3.5)
project(opennav_docking_core)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_ros_common REQUIRED)

nav2_package()

add_library(opennav_docking_core INTERFACE)
target_include_directories(opennav_docking_core
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(opennav_docking_core INTERFACE
  ${geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
  nav2_ros_common::nav2_ros_common
)

install(TARGETS opennav_docking_core
  EXPORT opennav_docking_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs
  nav2_ros_common
  tf2_ros
)
ament_export_targets(opennav_docking_core)

ament_package()
