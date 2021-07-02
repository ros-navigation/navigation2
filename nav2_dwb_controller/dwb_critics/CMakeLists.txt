cmake_minimum_required(VERSION 3.5)
project(dwb_critics)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(costmap_queue REQUIRED)
find_package(dwb_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_util REQUIRED)

nav2_package()

include_directories(
  include
)

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

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

set(dependencies
  angles
  nav2_costmap_2d
  costmap_queue
  dwb_core
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  pluginlib
  rclcpp
  sensor_msgs
  nav2_util
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include/
)
install(FILES default_critics.xml
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ${dependencies}
)
ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(dwb_core default_critics.xml)

ament_package()
