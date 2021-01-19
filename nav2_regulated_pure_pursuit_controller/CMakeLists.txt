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

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)

add_library(nav2_regulated_pure_pursuit_controller SHARED
        src/regulated_pure_pursuit_controller.cpp)

# prevent pluginlib from using boost
target_compile_definitions(nav2_regulated_pure_pursuit_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(nav2_regulated_pure_pursuit_controller
  ${dependencies}
)

install(TARGETS nav2_regulated_pure_pursuit_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_regulated_pure_pursuit_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_regulated_pure_pursuit_controller.xml)

ament_package()

