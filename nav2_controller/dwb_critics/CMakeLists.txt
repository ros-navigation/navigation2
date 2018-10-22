cmake_minimum_required(VERSION 3.5)
project(dwb_critics)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
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
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
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

pluginlib_export_plugin_description_file(dwb_core default_critics.xml)

ament_package()
