cmake_minimum_required(VERSION 3.5)
project(dwb_critics)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(costmap_queue REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

nav2_package()

add_library(${PROJECT_NAME} SHARED
  src/alignment_util.cpp
  src/map_grid.cpp
  src/goal_dist.cpp
  src/path_dist.cpp
  src/goal_align.cpp
  src/path_align.cpp
  src/base_obstacle.cpp
  src/obstacle_footprint.cpp
  src/oscillation.cpp
  src/prefer_forward.cpp
  src/rotate_to_goal.cpp
  src/twirling.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  costmap_queue::costmap_queue
  dwb_core::dwb_core
  ${dwb_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_2d_msgs_TARGETS}
  rclcpp::rclcpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  angles::angles
  nav2_util::nav2_util_core
  nav_2d_utils::path_ops
  pluginlib::pluginlib
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(FILES default_critics.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  costmap_queue
  dwb_core
  dwb_msgs
  geometry_msgs
  nav2_costmap_2d
  nav_2d_msgs
  rclcpp
)
ament_export_targets(${PROJECT_NAME})

pluginlib_export_plugin_description_file(dwb_core default_critics.xml)

ament_package()
