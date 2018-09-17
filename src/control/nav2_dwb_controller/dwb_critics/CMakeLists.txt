cmake_minimum_required(VERSION 3.5)
project(dwb_critics)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;-Wall")

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(costmap_2d REQUIRED)
find_package(costmap_queue REQUIRED)
find_package(dwb_local_planner REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}
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
ament_target_dependencies(${PROJECT_NAME}
  angles
  costmap_2d
  costmap_queue
  dwb_local_planner
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  pluginlib
  rclcpp
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include/
)
install(FILES default_critics.xml
        DESTINATION share
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  angles
  costmap_2d
  costmap_queue
  dwb_local_planner
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  pluginlib
  rclcpp
  sensor_msgs)
pluginlib_export_plugin_description_file(dwb_local_planner default_critics.xml)

ament_package()
