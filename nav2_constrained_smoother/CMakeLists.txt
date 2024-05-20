cmake_minimum_required(VERSION 3.5)
project(nav2_constrained_smoother)

set(CMAKE_BUILD_TYPE Release) # significant Ceres optimization speedup

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(tf2_ros REQUIRED)

if(${CERES_VERSION} VERSION_LESS_EQUAL 2.0.0)
  add_definitions(-DUSE_OLD_CERES_API)
endif()

nav2_package()

set(library_name nav2_constrained_smoother)

add_library(${library_name} SHARED src/constrained_smoother.cpp)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  nav2_core::nav2_core
  rclcpp::rclcpp
  nav2_util::nav2_util_core
  ${geometry_msgs_TARGETS}
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  nav2_costmap_2d::nav2_costmap_2d_core
  Ceres::ceres
  Eigen3::Eigen
)
target_link_libraries(${library_name} PRIVATE
  pluginlib::pluginlib
  ${nav_msgs_TARGETS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(angles REQUIRED)

  ament_find_gtest()

  add_subdirectory(test)
endif()

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(nav2_core rclcpp nav2_util geometry_msgs rclcpp_lifecycle tf2_ros nav2_costmap_2d ceres eigen3)
ament_export_targets(${library_name})

pluginlib_export_plugin_description_file(nav2_core nav2_constrained_smoother.xml)

ament_package()
