cmake_minimum_required(VERSION 3.5)
project(nav2_localization)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(BFL REQUIRED orocos-bfl)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  pluginlib
  nav2_util
  nav_2d_utils
  message_filters
  sensor_msgs
  tf2_ros
)

include_directories(
  include
  ${BFL_INCLUDE_DIRS}
)

link_directories(${BFL_LIBRARY_DIRS})

add_library(sample_motion_model src/plugins/sample_motion_model.cpp)

ament_target_dependencies(sample_motion_model ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_localization plugins.xml)

ament_package()
