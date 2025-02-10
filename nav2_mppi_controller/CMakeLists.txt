cmake_minimum_required(VERSION 3.5)
project(nav2_mppi_controller)

set(dependencies_pkgs
angles
geometry_msgs
nav2_common
nav2_core
nav2_costmap_2d
nav2_util
nav_msgs
pluginlib
rclcpp
rclcpp_lifecycle
std_msgs
tf2
tf2_geometry_msgs
tf2_ros
visualization_msgs
Eigen3
)
foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

nav2_package()

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()

# If building one the same hardware to be deployed on, try `-march=native`!

add_library(mppi_controller SHARED
  src/controller.cpp
  src/critic_manager.cpp
  src/noise_generator.cpp
  src/optimizer.cpp
  src/parameters_handler.cpp
  src/path_handler.cpp
  src/trajectory_visualizer.cpp
)

add_library(mppi_critics SHARED
  src/critics/constraint_critic.cpp
  src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/obstacles_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/velocity_deadband_critic.cpp
)

set(libraries mppi_controller mppi_critics)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts -O3)
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()


install(TARGETS mppi_controller mppi_critics
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
  # add_subdirectory(benchmark)
endif()

ament_export_libraries(${libraries})
ament_export_dependencies(
  angles
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  Eigen3
)
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core mppic.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

ament_package()
