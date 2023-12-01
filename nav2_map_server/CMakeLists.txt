cmake_minimum_required(VERSION 3.5)
project(nav2_map_server)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

pkg_search_module(UUID REQUIRED uuid)

nav2_package()

set(map_server_executable map_server)

set(library_name ${map_server_executable}_core)

set(map_io_library_name map_io)

set(vo_library_name vector_object_core)

set(map_saver_cli_executable map_saver_cli)

set(map_saver_server_executable map_saver_server)

set(costmap_filter_info_server_executable costmap_filter_info_server)

set(vo_server_executable vector_object_server)

add_library(${map_io_library_name} SHARED
  src/map_mode.cpp
  src/map_io.cpp)
target_include_directories(${map_io_library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_include_directories(${map_io_library_name}
  PRIVATE
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})
target_link_libraries(${map_io_library_name} PUBLIC
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
)
target_link_libraries(${map_io_library_name} PRIVATE
  ${GRAPHICSMAGICKCPP_LIBRARIES}
  tf2::tf2
  yaml-cpp::yaml-cpp
)

add_library(${library_name} SHARED
  src/map_server/map_server.cpp
  src/map_saver/map_saver.cpp
  src/costmap_filter_info/costmap_filter_info_server.cpp)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${library_name} PUBLIC
  ${map_io_library_name}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_link_libraries(${library_name} PRIVATE
  ${lifecycle_msgs_TARGETS}
  rclcpp_components::component
  yaml-cpp::yaml-cpp
)

add_library(${vo_library_name} SHARED
  src/vo_server/vector_object_shapes.cpp
  src/vo_server/vector_object_server.cpp)
target_include_directories(${vo_library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${vo_library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  tf2_ros::tf2_ros
  ${UUID_LIBRARIES}
)
target_link_libraries(${library_name} PRIVATE
  ${lifecycle_msgs_TARGETS}
  rclcpp_components::component
  yaml-cpp::yaml-cpp
)

add_executable(${map_server_executable}
  src/map_server/main.cpp)
target_include_directories(${map_server_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${map_server_executable} PRIVATE
  ${library_name}
  ${map_io_library_name}
  rclcpp::rclcpp
)

add_executable(${map_saver_cli_executable}
  src/map_saver/main_cli.cpp)
target_include_directories(${map_saver_cli_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${map_saver_cli_executable} PRIVATE
  ${library_name}
  ${map_io_library_name}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

add_executable(${map_saver_server_executable}
  src/map_saver/main_server.cpp)
target_include_directories(${map_saver_server_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${map_saver_server_executable} PRIVATE
  ${library_name}
  ${map_io_library_name}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

add_executable(${costmap_filter_info_server_executable}
  src/costmap_filter_info/main.cpp)
target_include_directories(${costmap_filter_info_server_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${costmap_filter_info_server_executable} PRIVATE
  ${library_name}
  ${map_io_library_name}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

add_executable(${vo_server_executable}
  src/vo_server/vector_object_server_node.cpp)
target_include_directories(${vo_server_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${vo_server_executable} PRIVATE
  ${vo_library_name}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

rclcpp_components_register_nodes(${library_name} "nav2_map_server::CostmapFilterInfoServer")
rclcpp_components_register_nodes(${library_name} "nav2_map_server::MapSaver")
rclcpp_components_register_nodes(${library_name} "nav2_map_server::MapServer")
rclcpp_components_register_nodes(${vo_library_name} "nav2_map_server::VectorObjectServer")

install(TARGETS
    ${library_name} ${map_io_library_name} ${vo_library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
    ${map_server_executable} ${map_saver_cli_executable} ${map_saver_server_executable}
    ${costmap_filter_info_server_executable} ${vo_server_executable}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(
  ${library_name}
  ${map_io_library_name}
  ${vo_library_name}
)
ament_export_dependencies(nav_msgs nav2_msgs nav2_util rclcpp rclcpp_lifecycle)
ament_export_targets(${library_name})

ament_package()
