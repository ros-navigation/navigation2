cmake_minimum_required(VERSION 3.5)
project(nav2_costmap_2d)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util)
find_package(nav2_voxel_grid REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)

remove_definitions(-DDISABLE_LIBUSB-1.0)

nav2_package()

add_library(nav2_costmap_2d_core SHARED
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
  src/clear_costmap_service.cpp
  src/footprint_collision_checker.cpp
  plugins/costmap_filters/costmap_filter.cpp
)
target_include_directories(nav2_costmap_2d_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav2_costmap_2d_core PUBLIC
  ${geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  ${tf2_sensor_msgs_TARGETS}
  ${map_msgs_TARGETS}
  ${std_srvs_TARGETS}
  pluginlib::pluginlib
)

add_library(layers SHARED
  plugins/inflation_layer.cpp
  plugins/static_layer.cpp
  plugins/obstacle_layer.cpp
  src/observation_buffer.cpp
  plugins/voxel_layer.cpp
  plugins/range_sensor_layer.cpp
  plugins/denoise_layer.cpp
)
target_include_directories(layers
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(layers PUBLIC
  angles::angles
  rclcpp::rclcpp
  message_filters::message_filters
  laser_geometry::laser_geometry
  nav2_voxel_grid::voxel_grid
  nav2_costmap_2d_core
)
target_link_libraries(layers PRIVATE
  pluginlib::pluginlib
)

add_library(filters SHARED
  plugins/costmap_filters/keepout_filter.cpp
  plugins/costmap_filters/speed_filter.cpp
  plugins/costmap_filters/binary_filter.cpp
)
target_include_directories(filters
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(filters PUBLIC
  nav2_costmap_2d_core
)

add_library(nav2_costmap_2d_client SHARED
  src/footprint_subscriber.cpp
  src/costmap_subscriber.cpp
  src/costmap_topic_collision_checker.cpp
)
target_include_directories(nav2_costmap_2d_client
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav2_costmap_2d_client PUBLIC
  nav2_costmap_2d_core
)

add_executable(nav2_costmap_2d_markers src/costmap_2d_markers.cpp)
target_link_libraries(nav2_costmap_2d_markers PRIVATE
  rclcpp::rclcpp
  ${visualization_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_voxel_grid::voxel_grid
  nav2_util::nav2_util_core
)

add_executable(nav2_costmap_2d_cloud src/costmap_2d_cloud.cpp)
target_link_libraries(nav2_costmap_2d_cloud PRIVATE
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  nav2_voxel_grid::voxel_grid
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
)

add_executable(nav2_costmap_2d src/costmap_2d_node.cpp)
target_link_libraries(nav2_costmap_2d PRIVATE
  rclcpp::rclcpp
  nav2_costmap_2d_core
)

install(TARGETS
  layers
  filters
  nav2_costmap_2d_core
  nav2_costmap_2d_client
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS nav2_costmap_2d_markers nav2_costmap_2d_cloud nav2_costmap_2d
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()
  add_subdirectory(test)
  pluginlib_export_plugin_description_file(nav2_costmap_2d test/regression/order_layer.xml)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs tf2_ros nav2_msgs nav2_util nav_msgs tf2_sensor_msgs map_msgs std_srvs pluginlib angles rclcpp message_filters laser_geometry nav2_voxel_grid)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_package()
