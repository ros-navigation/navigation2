cmake_minimum_required(VERSION 3.5)
project(nav2_constrained_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

nav2_package()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(library_name nav2_constrained_controller)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_2d_utils
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  pluginlib
  angles
)

add_library(${library_name} SHARED
  src/constrained_controller.cpp
  src/nominal_controller.cpp
  src/path_handler.cpp
  src/esdf_grid.cpp
  src/cbf_safety_filter.cpp
  src/qp_solver.cpp
  src/parameter_handler.cpp
  src/logger.cpp
)

target_compile_features(${library_name} PUBLIC cxx_std_17)
target_link_libraries(${library_name} Eigen3::Eigen)

ament_target_dependencies(${library_name} ${dependencies})

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES constrained_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_dependencies(Eigen3)

pluginlib_export_plugin_description_file(nav2_core constrained_controller_plugin.xml)

ament_package()
