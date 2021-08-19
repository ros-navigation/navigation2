cmake_minimum_required(VERSION 3.5)
project(nav2_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
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

add_library(${library_name}
  src/nav2_controller.cpp
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

set(dependencies
  angles
  rclcpp
  rclcpp_action
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
# prevent pluginlib from using boost
target_compile_definitions(simple_progress_checker PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(simple_goal_checker SHARED plugins/simple_goal_checker.cpp)
ament_target_dependencies(simple_goal_checker ${dependencies})
# prevent pluginlib from using boost
target_compile_definitions(simple_goal_checker PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(stopped_goal_checker SHARED plugins/stopped_goal_checker.cpp)
target_link_libraries(stopped_goal_checker simple_goal_checker)
ament_target_dependencies(stopped_goal_checker ${dependencies})
# prevent pluginlib from using boost
target_compile_definitions(stopped_goal_checker PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${library_name}
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

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

install(TARGETS simple_progress_checker simple_goal_checker stopped_goal_checker ${library_name}
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
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(simple_progress_checker
  simple_goal_checker
  stopped_goal_checker
  ${library_name})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

ament_package()
