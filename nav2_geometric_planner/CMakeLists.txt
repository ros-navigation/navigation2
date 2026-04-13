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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

# Common link targets shared by all three planner libraries.
set(PLANNER_LINK_LIBS
  nav2_core::nav2_core
  nav2_ros_common::nav2_ros_common
  nav2_costmap_2d::nav2_costmap_2d_core
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_ros::tf2_ros
  pluginlib::pluginlib
)

# ---------------------------------------------------------------------------
# CubicSplinePlanner
# ---------------------------------------------------------------------------
add_library(cubic_spline_planner SHARED
  CubicSplinePlanner/src/cubic_spline_planner.cpp
)
target_include_directories(cubic_spline_planner PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/CubicSplinePlanner/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(cubic_spline_planner PUBLIC ${PLANNER_LINK_LIBS})

# ---------------------------------------------------------------------------
# ClothoidPlanner
# ---------------------------------------------------------------------------
add_library(clothoid_planner SHARED
  ClothoidPlanner/src/clothoid_planner.cpp
)
target_include_directories(clothoid_planner PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ClothoidPlanner/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(clothoid_planner PUBLIC ${PLANNER_LINK_LIBS})

# ---------------------------------------------------------------------------
# BezierPlanner
# ---------------------------------------------------------------------------
add_library(bezier_planner SHARED
  BezierPlanner/src/bezier_planner.cpp
)
target_include_directories(bezier_planner PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/BezierPlanner/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(bezier_planner PUBLIC ${PLANNER_LINK_LIBS})

# ---------------------------------------------------------------------------
# Plugin registration
# ---------------------------------------------------------------------------
pluginlib_export_plugin_description_file(nav2_core geometric_planner_plugins.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS cubic_spline_planner clothoid_planner bezier_planner
  EXPORT nav2_geometric_planners
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  CubicSplinePlanner/include/
  ClothoidPlanner/include/
  BezierPlanner/include/
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

  add_subdirectory(CubicSplinePlanner/test)
  add_subdirectory(ClothoidPlanner/test)
  add_subdirectory(BezierPlanner/test)
endif()

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(cubic_spline_planner clothoid_planner bezier_planner)
ament_export_targets(nav2_geometric_planners)
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_ros_common
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
)
ament_package()
