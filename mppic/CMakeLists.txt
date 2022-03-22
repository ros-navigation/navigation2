cmake_minimum_required(VERSION 3.5)
project(mppic)

find_package(ament_cmake REQUIRED)
find_package(xtensor REQUIRED)

set(dependencies_pkgs
  rclcpp
  nav2_common
  pluginlib
  tf2
  geometry_msgs
  visualization_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

nav2_package()

add_library(mppic SHARED
  src/controller.cpp
  src/critic_manager.cpp
  src/path_handler.cpp
  src/trajectory_visualizer.cpp
  src/optimizer.cpp
)

add_library(critics SHARED
  src/critics/approx_reference_trajectory_critic.cpp
  src/critics/reference_trajectory_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/obstacles_critic.cpp
)

set(libraries mppic critics)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts)
  target_include_directories(${lib} PUBLIC include)
  target_link_libraries(${lib} xtensor)
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

install(TARGETS mppic critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core mppic.xml)
pluginlib_export_plugin_description_file(mppic critics.xml)

ament_package()
