add_library(${library_name}
  costmap.cpp
  lifecycle_service_client.cpp
  string_utils.cpp
  robot_utils.cpp
  controller_utils.cpp
  odometry_utils.cpp
  array_parser.cpp
  path_utils.cpp
)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${library_name} PUBLIC
  bondcpp::bondcpp
  geometry_msgs::geometry_msgs
  lifecycle_msgs::lifecycle_msgs
  nav2_msgs::nav2_msgs
  nav_msgs::nav_msgs
  rcl_interfaces::rcl_interfaces
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  pluginlib::pluginlib
  nav2_ros_common::nav2_ros_common
  angles::angles
)
target_link_libraries(${library_name} PRIVATE
  ${bond_TARGETS}
)
target_include_directories(${library_name} PUBLIC
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_executable(lifecycle_bringup
  lifecycle_bringup_commandline.cpp
)
target_link_libraries(lifecycle_bringup PRIVATE ${library_name} rclcpp::rclcpp)

add_executable(base_footprint_publisher
  base_footprint_publisher.cpp
)
target_link_libraries(base_footprint_publisher PRIVATE ${library_name} rclcpp::rclcpp tf2_msgs::tf2_msgs)

install(TARGETS
  ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_bringup
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
