cmake_minimum_required(VERSION 3.5)
project(nav2_toolkit)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

nav2_package()

set(executable_name pose_saver_node)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/pose_saver_node.cpp
)

target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${library_name} PUBLIC
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_ros_common::nav2_ros_common
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_srvs_TARGETS}
  yaml-cpp
)

target_link_libraries(${library_name} PRIVATE
  rclcpp_components::component
)

add_executable(${executable_name}
  src/pose_saver_main.cpp
)

target_include_directories(${executable_name} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${executable_name} PRIVATE
  ${library_name}
  rclcpp::rclcpp
  nav2_ros_common::nav2_ros_common
)

# Base footprint publisher executable
add_executable(base_footprint_publisher
  src/base_footprint_publisher.cpp
)

target_include_directories(base_footprint_publisher PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(base_footprint_publisher PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
)

rclcpp_components_register_nodes(${library_name} "nav2_pose_saver::PoseSaverNode")

install(TARGETS ${library_name}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name} base_footprint_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_find_gtest()

  ament_add_gtest(test_pose_saver_node
    test/test_pose_saver_node.cpp
  )

  target_include_directories(test_pose_saver_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  )

  target_link_libraries(test_pose_saver_node
    ${library_name}
    yaml-cpp
  )

  ament_target_dependencies(test_pose_saver_node
    ament_index_cpp
    geometry_msgs
    nav2_msgs
    nav2_ros_common
    nav2_util
    rclcpp
    rclcpp_lifecycle
    std_srvs
  )
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  ament_index_cpp
  geometry_msgs
  nav2_msgs
  nav2_ros_common
  nav2_util
  rclcpp
  rclcpp_lifecycle
  std_srvs
  yaml-cpp
)
ament_export_targets(${PROJECT_NAME})

ament_package()