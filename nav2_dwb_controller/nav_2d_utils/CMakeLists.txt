cmake_minimum_required(VERSION 3.5)
project(nav_2d_utils)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bondcpp REQUIRED)

nav2_package()

add_library(conversions SHARED
  src/conversions.cpp)
target_include_directories(conversions
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(conversions PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_2d_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  nav2_ros_common::nav2_ros_common
)
target_link_libraries(conversions PRIVATE
  nav2_util::nav2_util_core
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_library(path_ops SHARED
  src/path_ops.cpp)
target_include_directories(path_ops
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(path_ops PUBLIC
  ${nav_2d_msgs_TARGETS}
  rclcpp::rclcpp
  nav2_ros_common::nav2_ros_common
  nav2_util::nav2_util_core
)
target_link_libraries(path_ops PRIVATE
  ${geometry_msgs_TARGETS}
)

add_library(tf_help SHARED
  src/tf_help.cpp
)
target_include_directories(tf_help
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(tf_help PUBLIC
  conversions
  ${geometry_msgs_TARGETS}
  ${nav_2d_msgs_TARGETS}
  rclcpp::rclcpp
  nav2_ros_common::nav2_ros_common
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  rclcpp_lifecycle::rclcpp_lifecycle
  rcl_lifecycle::rcl_lifecycle
)

install(TARGETS conversions path_ops tf_help
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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
ament_export_libraries(conversions path_ops tf_help)
ament_export_dependencies(
  geometry_msgs
  nav_2d_msgs
  bondcpp
  rclcpp_lifecycle
  rcl_lifecycle
  nav2_ros_common
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros)
ament_export_targets(${PROJECT_NAME})

ament_package()
