cmake_minimum_required(VERSION 3.5)
project(nav2_path_classifier)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(POLYCLIPPING REQUIRED polyclipping)

nav2_package()

include_directories(
  include
  ${POLYCLIPPING_INCLUDE_DIRS}
)

set(executable_name path_classifier_server)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  pluginlib
)

# Server core library
add_library(${library_name} SHARED
  src/path_classifier_server.cpp
  src/pose_classifier.cpp
  src/path_splitter.cpp
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

# Pose classifier plugins
add_library(pose_classifiers SHARED
  src/classifiers/confined_space_classifier.cpp
)

ament_target_dependencies(pose_classifiers
  ${dependencies}
)

target_link_libraries(pose_classifiers
  ${POLYCLIPPING_LIBRARIES}
)

pluginlib_export_plugin_description_file(nav2_path_classifier plugins.xml)

rclcpp_components_register_nodes(${library_name} "nav2_path_classifier::PathClassifierServer")

install(TARGETS ${library_name} pose_classifiers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
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
ament_export_libraries(${library_name} pose_classifiers)
ament_export_dependencies(${dependencies})
ament_package()
