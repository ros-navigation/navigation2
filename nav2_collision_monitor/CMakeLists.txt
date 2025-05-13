cmake_minimum_required(VERSION 3.5)
project(nav2_collision_monitor)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

nav2_package()

set(monitor_executable_name collision_monitor)
set(detector_executable_name collision_detector)
set(monitor_library_name ${monitor_executable_name}_core)
set(detector_library_name ${detector_executable_name}_core)

add_library(${monitor_library_name} SHARED
  src/collision_monitor_node.cpp
  src/polygon.cpp
  src/velocity_polygon.cpp
  src/circle.cpp
  src/source.cpp
  src/scan.cpp
  src/pointcloud.cpp
  src/polygon_source.cpp
  src/range.cpp
  src/kinematics.cpp
)
target_include_directories(${monitor_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${monitor_library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${monitor_library_name} PRIVATE
  rclcpp_components::component
)

add_library(${detector_library_name} SHARED
  src/collision_detector_node.cpp
  src/polygon.cpp
  src/velocity_polygon.cpp
  src/circle.cpp
  src/source.cpp
  src/scan.cpp
  src/pointcloud.cpp
  src/polygon_source.cpp
  src/range.cpp
  src/kinematics.cpp
)
target_include_directories(${detector_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${detector_library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2::tf2
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${detector_library_name} PRIVATE
  rclcpp_components::component
)

add_executable(${monitor_executable_name}
  src/collision_monitor_main.cpp
)
target_include_directories(${monitor_executable_name}
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${monitor_executable_name} PRIVATE
  rclcpp::rclcpp
  ${monitor_library_name}
)

add_executable(${detector_executable_name}
  src/collision_detector_main.cpp
)
target_include_directories(${detector_executable_name}
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${detector_executable_name} PRIVATE
  rclcpp::rclcpp
  ${detector_library_name}
)

rclcpp_components_register_nodes(${monitor_library_name} "nav2_collision_monitor::CollisionMonitor")

rclcpp_components_register_nodes(${detector_library_name} "nav2_collision_monitor::CollisionDetector")

install(TARGETS ${monitor_library_name} ${detector_library_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${monitor_executable_name} ${detector_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_find_gtest()

  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${monitor_library_name} ${detector_library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
