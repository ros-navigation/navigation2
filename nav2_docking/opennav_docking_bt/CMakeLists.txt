cmake_minimum_required(VERSION 3.8)
project(opennav_docking_bt)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_ros_common REQUIRED)

nav2_package()

add_library(opennav_dock_action_bt_node SHARED src/dock_robot.cpp)
target_include_directories(opennav_dock_action_bt_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_definitions(opennav_dock_action_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(opennav_dock_action_bt_node PUBLIC
  behaviortree_cpp::behaviortree_cpp
  ${geometry_msgs_TARGETS}
  nav2_behavior_tree::nav2_behavior_tree
  nav2_ros_common::nav2_ros_common
  ${nav2_msgs_TARGETS}
)

add_library(opennav_undock_action_bt_node SHARED src/undock_robot.cpp)
target_include_directories(opennav_undock_action_bt_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_definitions(opennav_undock_action_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(opennav_undock_action_bt_node PUBLIC
  behaviortree_cpp::behaviortree_cpp
  ${geometry_msgs_TARGETS}
  nav2_behavior_tree::nav2_behavior_tree
  nav2_ros_common::nav2_ros_common
  ${nav2_msgs_TARGETS}
)

install(TARGETS opennav_dock_action_bt_node opennav_undock_action_bt_node
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(opennav_dock_action_bt_node opennav_undock_action_bt_node)
ament_export_dependencies(
  behaviortree_cpp
  geometry_msgs
  nav2_behavior_tree
  nav2_ros_common
  nav2_msgs
)
ament_export_targets(${PROJECT_NAME})
ament_package()
