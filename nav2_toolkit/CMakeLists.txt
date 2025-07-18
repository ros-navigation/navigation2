cmake_minimum_required(VERSION 3.14)
project(nav2_toolkit)

find_package(nav2_common REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(pose_saver_library_name ${PROJECT_NAME}_core)
set(pose_saver_executable_name pose_saver)

add_library(${pose_saver_library_name} SHARED
  src/pose_saver_node.cpp
)
target_include_directories(${pose_saver_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${pose_saver_library_name} PUBLIC
  rclcpp::rclcpp
  rclcpp_components::rclcpp_components
  geometry_msgs::geometry_msgs
  std_srvs::std_srvs
  yaml-cpp
  ament_index_cpp::ament_index_cpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)
target_link_libraries(${pose_saver_library_name} PRIVATE
  rclcpp_components::component
)

add_executable(${pose_saver_executable_name}
  src/pose_saver_main.cpp
)

rclcpp_components_register_nodes(${pose_saver_library_name} "nav2_toolkit::PoseSaverNode")

install(TARGETS ${pose_saver_library_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${pose_saver_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pose_saver_node
    test/test_pose_saver_node.cpp
  )

  target_include_directories(test_pose_saver_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_pose_saver_node
    pose_saver_component
    yaml-cpp
  )

  ament_target_dependencies(test_pose_saver_node
    rclcpp
    geometry_msgs
    nav2_msgs
    std_srvs
    nav2_util
  )
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${pose_saver_library_name})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
  std_srvs
  yaml-cpp
  ament_index_cpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
