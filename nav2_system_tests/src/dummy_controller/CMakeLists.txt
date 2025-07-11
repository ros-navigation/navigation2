add_executable(dummy_controller_node
  src/dummy_controller/main.cpp
  src/dummy_controller/dummy_controller.cpp
)
target_link_libraries(dummy_controller_node PRIVATE
  ${geometry_msgs_TARGETS}
  nav2_behavior_tree::nav2_behavior_tree
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  nav2_ros_common::nav2_ros_common
)
target_include_directories(dummy_controller_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
