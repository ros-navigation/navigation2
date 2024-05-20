cmake_minimum_required(VERSION 3.5)
project(nav2_regulated_pure_pursuit_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
)

set(library_name nav2_regulated_pure_pursuit_controller)

add_library(${library_name} SHARED
  src/regulated_pure_pursuit_controller.cpp
  src/collision_checker.cpp
  src/parameter_handler.cpp
  src/path_handler.cpp)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp::rclcpp
  nav2_costmap_2d::nav2_costmap_2d_core
  ${geometry_msgs_TARGETS}
  pluginlib::pluginlib
  nav2_core::nav2_core
  nav2_util::nav2_util_core
  tf2_ros::tf2_ros
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
)

install(TARGETS ${library_name}
  EXPORT ${library_name}
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

  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_targets(${library_name})

pluginlib_export_plugin_description_file(nav2_core nav2_regulated_pure_pursuit_controller.xml)

ament_package()
