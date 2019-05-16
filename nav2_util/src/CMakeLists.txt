add_library(${library_name} SHARED
  costmap.cpp
  node_utils.cpp
  lifecycle_service_client.cpp
  string_utils.cpp
  lifecycle_utils.cpp
<<<<<<< HEAD
  lifecycle_node.cpp
  src/robot_utils.cpp
=======
  robot_utils.cpp
>>>>>>> resolve merge conflicts
)

ament_target_dependencies(${library_name}
  rclcpp
  nav2_msgs
  tf2
  tf2_ros
  nav_msgs
  geometry_msgs
  lifecycle_msgs
<<<<<<< HEAD
  rclcpp_lifecycle
=======
>>>>>>> resolve merge conflicts
)

add_subdirectory(pf)
add_subdirectory(map)
add_subdirectory(map_loader)
add_subdirectory(motion_model)
add_subdirectory(sensors)

add_executable(lifecycle_bringup
  lifecycle_bringup_commandline.cpp
)
target_link_libraries(lifecycle_bringup ${library_name})
<<<<<<< HEAD

install(TARGETS
  ${library_name}
  lifecycle_bringup
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
=======
>>>>>>> resolve merge conflicts
