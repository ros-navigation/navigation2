cmake_minimum_required(VERSION 3.5)
project(nav2_localization)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(laser_geometry REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  pluginlib
  nav2_util
  nav_2d_utils
  message_filters
  sensor_msgs
  tf2_ros
  tf2
  nav2_msgs
  laser_geometry
)

include_directories(include)

## Declare libraries
add_library(sample_motion_model SHARED
  src/plugins/motion_models/diff_drive_odom_motion_model.cpp
)

ament_target_dependencies(sample_motion_model
  ${dependencies}
)

add_library(matcher2d SHARED
  src/plugins/matchers/likelihood_field_matcher2d.cpp
  src/map_utils.cpp
)

ament_target_dependencies(matcher2d
  ${dependencies}
)

add_library(solver SHARED
  src/interfaces/particle_filter_base.cpp
  src/plugins/solvers/mcl.cpp
)

ament_target_dependencies(solver
  ${dependencies}
)

## Declare executable
add_executable(${PROJECT_NAME}
  src/nav2_localization.cpp  
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
  sample_motion_model
  matcher2d
  solver
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

################################################

install(TARGETS sample_motion_model matcher2d solver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
# endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(
  sample_motion_model
  matcher2d
  solver
)

pluginlib_export_plugin_description_file(nav2_localization plugins.xml)

ament_package()
