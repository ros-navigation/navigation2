cmake_minimum_required(VERSION 3.8)
project(nav2_segmentation)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

nav2_package()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SegmentImage.action"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(nav2_segment_image_action_bt_node SHARED
  src/segment_image_action.cpp
)
target_include_directories(nav2_segment_image_action_bt_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_definitions(nav2_segment_image_action_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(nav2_segment_image_action_bt_node PUBLIC
  behaviortree_cpp::behaviortree_cpp
  nav2_behavior_tree::nav2_behavior_tree
  nav2_ros_common::nav2_ros_common
  rclcpp_action::rclcpp_action
  ${cpp_typesupport_target}
)

add_executable(nav2_segmentation_server
  src/segmentation_server.cpp
)
target_link_libraries(nav2_segmentation_server
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${cpp_typesupport_target}
)

install(TARGETS
  nav2_segment_image_action_bt_node
  nav2_segmentation_server
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(nav2_segment_image_action_bt_node)
ament_export_dependencies(
  behaviortree_cpp
  nav2_behavior_tree
  nav2_ros_common
  rclcpp
  rclcpp_action
  rosidl_default_runtime
)
ament_export_targets(${PROJECT_NAME})

ament_package()
