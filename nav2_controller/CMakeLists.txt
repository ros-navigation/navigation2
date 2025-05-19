cmake_minimum_required(VERSION 3.5)
project(nav2_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name controller_server)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/controller_server.cpp
)

set(dependencies
  angles
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  nav2_msgs
  nav_2d_utils
  nav_2d_msgs
  nav2_util
  nav2_core
  pluginlib
)

add_library(simple_progress_checker SHARED plugins/simple_progress_checker.cpp)
ament_target_dependencies(simple_progress_checker ${dependencies})

add_library(pose_progress_checker SHARED plugins/pose_progress_checker.cpp)
target_link_libraries(pose_progress_checker simple_progress_checker)
ament_target_dependencies(pose_progress_checker ${dependencies})

add_library(simple_goal_checker SHARED plugins/simple_goal_checker.cpp)
ament_target_dependencies(simple_goal_checker ${dependencies})

add_library(stopped_goal_checker SHARED plugins/stopped_goal_checker.cpp)
target_link_libraries(stopped_goal_checker simple_goal_checker)
ament_target_dependencies(stopped_goal_checker ${dependencies})

add_library(position_goal_checker SHARED plugins/position_goal_checker.cpp)
target_link_libraries(position_goal_checker simple_goal_checker)
ament_target_dependencies(position_goal_checker ${dependencies})

ament_target_dependencies(${library_name}
  ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(plugins/test)
endif()

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "nav2_controller::ControllerServer")

install(TARGETS simple_progress_checker position_goal_checker pose_progress_checker simple_goal_checker stopped_goal_checker ${library_name}
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
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(simple_progress_checker
  pose_progress_checker
  simple_goal_checker
  stopped_goal_checker
  position_goal_checker
  ${library_name})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

ament_package()
