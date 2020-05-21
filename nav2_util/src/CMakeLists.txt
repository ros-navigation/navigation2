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
)

ament_target_dependencies(${library_name}
  rclcpp
  nav2_msgs
  tf2
  tf2_ros
  nav_msgs
  geometry_msgs
  lifecycle_msgs
  rclcpp_lifecycle
  tf2_geometry_msgs
)

add_executable(lifecycle_bringup
  lifecycle_bringup_commandline.cpp
)
target_link_libraries(lifecycle_bringup ${library_name})

find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(dump_params dump_params.cpp)
ament_target_dependencies(dump_params rclcpp)

target_include_directories(dump_params PUBLIC ${Boost_INCLUDE_DIRS})

target_link_libraries(dump_params ${Boost_PROGRAM_OPTIONS_LIBRARY})

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_bringup
  dump_params
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
