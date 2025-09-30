cmake_minimum_required(VERSION 3.5)
project(opennav_following_bt)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(opennav_following_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

nav2_package()

add_library(opennav_follow_action_bt_node SHARED src/follow_object_action.cpp)
target_include_directories(opennav_follow_action_bt_node
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
target_compile_definitions(opennav_follow_action_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(opennav_follow_action_bt_node PUBLIC
  behaviortree_cpp::behaviortree_cpp
  ${geometry_msgs_TARGETS}
  nav2_behavior_tree::nav2_behavior_tree
  ${opennav_following_msgs_TARGETS}
)

add_library(opennav_follow_cancel_bt_node SHARED src/follow_object_cancel_node.cpp)
target_include_directories(opennav_follow_cancel_bt_node
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
target_compile_definitions(opennav_follow_cancel_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(opennav_follow_cancel_bt_node PUBLIC
  behaviortree_cpp::behaviortree_cpp
  ${geometry_msgs_TARGETS}
  nav2_behavior_tree::nav2_behavior_tree
  ${opennav_following_msgs_TARGETS}
)

install(TARGETS opennav_follow_action_bt_node opennav_follow_cancel_bt_node
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
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(opennav_follow_action_bt_node follow_object_cancel_node)
ament_export_dependencies(
  behaviortree_cpp
  geometry_msgs
  nav2_behavior_tree
  nav2_ros_common
  opennav_following_msgs
)
ament_export_targets(${PROJECT_NAME})
ament_package()
