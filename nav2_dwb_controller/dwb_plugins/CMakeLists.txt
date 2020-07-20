cmake_minimum_required(VERSION 3.5)
project(dwb_plugins)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(dwb_core REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)

nav2_package()

set(dependencies
  angles
  dwb_core
  nav_2d_msgs
  nav_2d_utils
  pluginlib
  rclcpp
  nav2_util
)

include_directories(
  include
)

add_library(standard_traj_generator SHARED
            src/standard_traj_generator.cpp
            src/limited_accel_generator.cpp
            src/kinematic_parameters.cpp
            src/xy_theta_iterator.cpp)
ament_target_dependencies(standard_traj_generator ${dependencies})
# prevent pluginlib from using boost
target_compile_definitions(standard_traj_generator PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(TARGETS standard_traj_generator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include/
)
install(FILES plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(standard_traj_generator)
pluginlib_export_plugin_description_file(dwb_core plugins.xml)

ament_package()
