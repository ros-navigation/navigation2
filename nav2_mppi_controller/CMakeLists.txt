cmake_minimum_required(VERSION 3.5)
project(nav2_mppi_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

nav2_package()

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mno-avx512f" COMPILER_SUPPORTS_AVX512)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_AVX512)
  add_compile_options(-mno-avx512f)
endif()

if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()

if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()

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
target_compile_options(mppi_controller PUBLIC -fconcepts -O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -mtune=generic)
target_include_directories(mppi_controller
  PUBLIC
    ${xsimd_INCLUDE_DIRS}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(mppi_controller PUBLIC
  angles::angles
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
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
target_compile_options(mppi_critics PUBLIC -fconcepts -O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -mtune=generic)
target_include_directories(mppi_critics
  PUBLIC
    ${xsimd_INCLUDE_DIRS}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(mppi_critics PUBLIC
  angles::angles
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
target_link_libraries(mppi_critics PRIVATE
  pluginlib::pluginlib
)

install(TARGETS mppi_controller mppi_critics
  EXPORT nav2_mppi_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_find_gtest()

  add_subdirectory(test)
  add_subdirectory(benchmark)
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
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(nav2_mppi_controller)

pluginlib_export_plugin_description_file(nav2_core mppic.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

ament_package()
