cmake_minimum_required(VERSION 3.5)
project(nav2_map_server)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)

nav2_package()

set(map_server_executable map_server)

set(library_name ${map_server_executable}_core)

set(map_io_library_name map_io)

set(map_saver_cli_executable map_saver_cli)

set(map_saver_server_executable map_saver_server)

set(costmap_filter_info_server_executable costmap_filter_info_server)

add_library(${library_name} SHARED
  src/map_server/map_server.cpp
  src/map_saver/map_saver.cpp
  src/costmap_filter_info/costmap_filter_info_server.cpp)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${library_name} PUBLIC
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)
target_link_libraries(${library_name} PRIVATE
  ${lifecycle_msgs_TARGETS}
  rclcpp_components::component
  yaml-cpp
)

add_library(${map_io_library_name} SHARED
  src/map_mode.cpp
  src/map_io.cpp)
target_include_directories(${map_io_library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${map_io_library_name} PUBLIC
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
)
target_link_libraries(${map_io_library_name} PRIVATE
  tf2::tf2
  yaml-cpp
  ${GRAPHICSMAGICKCPP_LIBRARIES}
)
if(WIN32)
  target_compile_definitions(${map_io_library_name} PRIVATE
    YAML_CPP_DLL)
endif()

add_executable(${map_server_executable}
  src/map_server/main.cpp)
target_include_directories(${map_server_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${map_server_executable} PRIVATE
  rclcpp::rclcpp
  ${library_name}
  ${map_io_library_name}
)
if(WIN32)
  target_compile_definitions(${map_server_executable} PRIVATE
    YAML_CPP_DLL)
endif()

add_executable(${map_saver_cli_executable}
  src/map_saver/main_cli.cpp)
target_include_directories(${map_saver_cli_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${map_saver_cli_executable} PRIVATE
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${library_name}
  ${map_io_library_name}
)

add_executable(${map_saver_server_executable}
  src/map_saver/main_server.cpp)
target_include_directories(${map_saver_server_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${map_saver_server_executable} PRIVATE
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${library_name}
  ${map_io_library_name}
)

add_executable(${costmap_filter_info_server_executable}
  src/costmap_filter_info/main.cpp)
target_include_directories(${costmap_filter_info_server_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${costmap_filter_info_server_executable} PRIVATE
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${library_name}
  ${map_io_library_name}
)

rclcpp_components_register_nodes(${library_name} "nav2_map_server::CostmapFilterInfoServer")
rclcpp_components_register_nodes(${library_name} "nav2_map_server::MapSaver")
rclcpp_components_register_nodes(${library_name} "nav2_map_server::MapServer")

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ${map_io_library_name}
  EXPORT ${map_io_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
    ${map_server_executable} ${map_saver_cli_executable} ${map_saver_server_executable}
    ${costmap_filter_info_server_executable}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${library_name}
  ${map_io_library_name}
)
ament_export_dependencies(nav2_util rclcpp rclcpp_lifecycle nav2_msgs nav_msgs)
ament_export_targets(${library_name} ${map_io_library_name})
ament_package()
