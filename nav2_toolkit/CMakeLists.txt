cmake_minimum_required(VERSION 3.5)
project(nav2_toolkit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

nav2_package()

set(executable_name pose_saver_node)
set(saver_library_name ${PROJECT_NAME}_core)

add_library(${saver_library_name} SHARED
  src/pose_saver_node.cpp
)
target_include_directories(${saver_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${saver_library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
  yaml-cpp::yaml-cpp
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
)
target_link_libraries(${saver_library_name} PRIVATE
  rclcpp_components::component
)
rclcpp_components_register_node(${saver_library_name}
  PLUGIN "nav2_toolkit::PoseSaverNode"
  EXECUTABLE ${executable_name}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${saver_library_name})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_srvs
  yaml-cpp
  ament_index_cpp
)
ament_export_targets(${saver_library_name})
ament_package()
