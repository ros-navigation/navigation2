cmake_minimum_required(VERSION 3.14)
project(nav2_toolkit)

find_package(nav2_common REQUIRED)
nav2_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)


include_directories(include/nav2_toolkit)

add_library(pose_saver_component SHARED
  src/pose_saver_node.cpp
)

target_include_directories(pose_saver_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(pose_saver_component
  rclcpp
  rclcpp_components
  geometry_msgs
  std_srvs
  yaml-cpp
  ament_index_cpp
  nav2_msgs
  nav2_util
  nav2_ros_common
)

target_link_libraries(pose_saver_component
  yaml-cpp
)

rclcpp_components_register_nodes(pose_saver_component
  "nav2_pose_saver::PoseSaverNode"
)

add_executable(pose_saver_node
  src/pose_saver_main.cpp
)

target_link_libraries(pose_saver_node
  pose_saver_component
)

ament_target_dependencies(pose_saver_node
  rclcpp
  nav2_msgs
  nav2_ros_common
)

install(TARGETS
  pose_saver_component
  pose_saver_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(pose_saver_component)
ament_export_targets(export_${PROJECT_NAME})

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
    nav2_ros_common
  )
endif()

ament_package()
