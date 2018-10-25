cmake_minimum_required(VERSION 3.5)
project(nav2_costmap_2d)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(xmlrpcpp REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pluginlib REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_util REQUIRED)

# TODO(bpwilcox): port remaining packages
#find_package(dynamic_reconfigure REQUIRED)
#find_package(voxel_grid REQUIRED)

remove_definitions(-DDISABLE_LIBUSB-1.0)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

# TODO(bpwilcox): Separate msgs generation into separate package
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/VoxelGrid.msg"
#
#	DEPENDENCIES std_msgs geometry_msgs map_msgs
#)

add_library(nav2_costmap_2d SHARED
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_math.cpp
  src/footprint.cpp
  src/costmap_layer.cpp
  src/observation_buffer.cpp
)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  map_msgs
  tf2_ros
  tf2
  xmlrpcpp
  pluginlib
  tf2_geometry_msgs
  tf2_sensor_msgs
  message_filters
  nav2_util
)

ament_target_dependencies(nav2_costmap_2d
  ${dependencies}
)

add_library(layers SHARED
  plugins/inflation_layer.cpp
  plugins/static_layer.cpp
  #plugins/obstacle_layer.cpp
  src/observation_buffer.cpp

  # TODO(bpwilcox): port additional plugin layers
  #plugins/voxel_layer.cpp
)
ament_target_dependencies(layers
  ${dependencies}
 )
 target_link_libraries(layers
  nav2_costmap_2d
)

# TODO(bpwilcox): port nav2_costmap_2d_markers
#add_executable(nav2_costmap_2d_markers src/costmap_2d_markers.cpp)
#ament_target_dependencies(nav2_costmap_2d_markers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${ament_EXPORTED_TARGETS})
#target_link_libraries(nav2_costmap_2d_markers
  #nav2_costmap_2d
#)

# TODO(bpwilcox): port nav2_costmap_2d_cloud
#add_executable(nav2_costmap_2d_cloud src/costmap_2d_cloud.cpp)
#ament_target_dependencies(nav2_costmap_2d_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${ament_EXPORTED_TARGETS})
#target_link_libraries(nav2_costmap_2d_cloud
  #nav2_costmap_2d
#)

add_executable(nav2_costmap_2d_node src/costmap_2d_node.cpp)
ament_target_dependencies(nav2_costmap_2d_node
  ${dependencies}
)

target_link_libraries(nav2_costmap_2d_node
  nav2_costmap_2d
)

install(TARGETS nav2_costmap_2d layers nav2_costmap_2d_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # TODO(bpwilcox): Port/Add unit and component tests
  #add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_costmap_2d layers)
ament_export_dependencies(${dependencies})
ament_package()
