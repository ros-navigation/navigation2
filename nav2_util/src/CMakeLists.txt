add_library(${library_name} SHARED
  costmap.cpp
  lifecycle_service_client.cpp
  string_utils.cpp
  robot_utils.cpp
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
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
  pluginlib::pluginlib
)
target_link_libraries(${library_name} PRIVATE
  ${bond_TARGETS}
)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_executable(lifecycle_bringup
  lifecycle_bringup_commandline.cpp
)
target_link_libraries(lifecycle_bringup PRIVATE ${library_name} rclcpp::rclcpp)

add_executable(base_footprint_publisher
  base_footprint_publisher.cpp
)
target_link_libraries(base_footprint_publisher PRIVATE ${library_name} rclcpp::rclcpp ${tf2_msgs_TARGETS})

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
