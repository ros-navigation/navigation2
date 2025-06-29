cmake_minimum_required(VERSION 3.5)
project(nav2_theta_star_planner)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_ros_common REQUIRED)

nav2_package()

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/theta_star.cpp
  src/theta_star_planner.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>")
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  pluginlib::pluginlib
)
target_compile_options(${library_name} PRIVATE -O3)

pluginlib_export_plugin_description_file(nav2_core theta_star_planner.xml)

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES theta_star_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(gtest_disable_pthreads OFF)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_find_gtest()

  ament_add_gtest(test_theta_star test/test_theta_star.cpp)
  target_link_libraries(test_theta_star
    ${library_name}
    ${geometry_msgs_TARGETS}
    nav2_costmap_2d::nav2_costmap_2d_core
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )
endif()


ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2dnav2_util
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  nav2_ros_common
  rcl_interfaces
  tf2_ros
)
ament_export_targets(${library_name})
ament_package()
