cmake_minimum_required(VERSION 3.8)
project(nav2_geometric_planners)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(PLANNER_LINK_LIBS
  nav2_core::nav2_core
  nav2_ros_common::nav2_ros_common
  nav2_costmap_2d::nav2_costmap_2d_core
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  pluginlib::pluginlib
)

# ---------------------------------------------------------------------------
# Single plugin library
# ---------------------------------------------------------------------------
add_library(geometric_planner SHARED
  src/geometric_planner.cpp
  src/bezier_solver.cpp
  src/cubic_spline_solver.cpp
  src/clothoid_solver.cpp
)
target_include_directories(geometric_planner PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/math>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(geometric_planner PUBLIC ${PLANNER_LINK_LIBS})

# ---------------------------------------------------------------------------
# Plugin registration
# ---------------------------------------------------------------------------
pluginlib_export_plugin_description_file(nav2_core geometric_planner_plugins.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS geometric_planner
  EXPORT nav2_geometric_planners
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  math/
  utils/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES geometric_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_find_gtest()

  add_subdirectory(test)
endif()

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(geometric_planner)
ament_export_targets(nav2_geometric_planners)
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_ros_common
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)
ament_package()
