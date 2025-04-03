add_library(${library_name} SHARED
  costmap.cpp
  node_utils.cpp
  lifecycle_service_client.cpp
  string_utils.cpp
  lifecycle_utils.cpp
  lifecycle_node.cpp
  robot_utils.cpp
  node_thread.cpp
  odometry_utils.cpp
  array_parser.cpp
)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${library_name}
  bondcpp
  geometry_msgs
  lifecycle_msgs
  nav2_msgs
  nav_msgs
  rcl_interfaces
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2_ros
  tf2
  tf2_geometry_msgs
  bond
)


add_executable(lifecycle_bringup
  lifecycle_bringup_commandline.cpp
)
ament_target_dependencies(lifecycle_bringup
  rclcpp
)
target_link_libraries(lifecycle_bringup
  ${library_name}
)

## Commented because it is not possible to be used with Humble
# add_executable(base_footprint_publisher
#   base_footprint_publisher.cpp
# )
# ament_target_dependencies(base_footprint_publisher
#   rclcpp
#   tf2_msgs
# )
# target_link_libraries(base_footprint_publisher
#   ${library_name}
# )

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
