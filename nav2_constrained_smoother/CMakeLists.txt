cmake_minimum_required(VERSION 3.5)
project(nav2_constrained_smoother)

set(CMAKE_BUILD_TYPE Release) # significant Ceres optimization speedup

find_package(ament_cmake REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(library_name nav2_constrained_smoother)

add_library(${library_name} SHARED src/constrained_smoother.cpp)
if(${CERES_VERSION} VERSION_LESS_EQUAL 2.0.0)
  target_compile_definitions(${library_name} PUBLIC -DUSE_OLD_CERES_API)
endif()
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  Ceres::ceres
  Eigen3::Eigen
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
)

if(BUILD_TESTING)

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

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(ceres eigen3 nav2_core nav2_costmap_2d nav2_util rclcpp rclcpp_lifecycle tf2_ros)
ament_export_targets(${library_name})

pluginlib_export_plugin_description_file(nav2_core nav2_constrained_smoother.xml)

ament_package()
