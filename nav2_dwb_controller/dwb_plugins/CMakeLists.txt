cmake_minimum_required(VERSION 3.5)
project(dwb_plugins)

find_package(ament_cmake REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

nav2_package()

add_library(standard_traj_generator SHARED
  src/standard_traj_generator.cpp
  src/limited_accel_generator.cpp
  src/kinematic_parameters.cpp
  src/xy_theta_iterator.cpp)
target_include_directories(standard_traj_generator PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(standard_traj_generator PUBLIC
  dwb_core::dwb_core
  ${dwb_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav_2d_msgs_TARGETS}
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
)
target_link_libraries(standard_traj_generator PRIVATE
  nav_2d_utils::conversions
  pluginlib::pluginlib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

  find_package(nav2_costmap_2d REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  add_subdirectory(test)
endif()

install(TARGETS standard_traj_generator
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(standard_traj_generator)
ament_export_dependencies(
  dwb_core
  dwb_msgs
  geometry_msgs
  nav2_util
  nav_2d_msgs
  rclcpp
  rcl_interfaces
)
ament_export_targets(${PROJECT_NAME})

pluginlib_export_plugin_description_file(dwb_core plugins.xml)

ament_package()
