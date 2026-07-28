cmake_minimum_required(VERSION 3.5)
project(nav2_pose_classifiers)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(POLYCLIPPING REQUIRED polyclipping)

nav2_package()

include_directories(
  include
  ${POLYCLIPPING_INCLUDE_DIRS}
)

set(library_name nav2_pose_classifiers)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
  tf2_ros
  pluginlib
)

add_library(${library_name} SHARED
  src/constraint_classifier.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  ${POLYCLIPPING_LIBRARIES}
)

pluginlib_export_plugin_description_file(nav2_pose_classifiers pose_classifier_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
