cmake_minimum_required(VERSION 3.20)
project(nav2_map_server)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# ROS Humble compatibility shims
if(NOT TARGET geometry_msgs::geometry_msgs)
  add_library(geometry_msgs::geometry_msgs INTERFACE IMPORTED)
  target_link_libraries(geometry_msgs::geometry_msgs INTERFACE
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)
endif()

if(NOT TARGET nav_msgs::nav_msgs)
  add_library(nav_msgs::nav_msgs INTERFACE IMPORTED)
  target_link_libraries(nav_msgs::nav_msgs INTERFACE
    nav_msgs::nav_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp)
endif()

if(NOT TARGET nav2_msgs::nav2_msgs)
  add_library(nav2_msgs::nav2_msgs INTERFACE IMPORTED)
  target_link_libraries(nav2_msgs::nav2_msgs INTERFACE
    nav2_msgs::nav2_msgs__rosidl_generator_cpp
    nav2_msgs::nav2_msgs__rosidl_typesupport_cpp)
endif()

if(NOT TARGET lifecycle_msgs::lifecycle_msgs)
  add_library(lifecycle_msgs::lifecycle_msgs INTERFACE IMPORTED)
  target_link_libraries(lifecycle_msgs::lifecycle_msgs INTERFACE
    lifecycle_msgs::lifecycle_msgs__rosidl_generator_cpp
    lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp)
endif()

if(NOT TARGET map_msgs::map_msgs)
  add_library(map_msgs::map_msgs INTERFACE IMPORTED)
  target_link_libraries(map_msgs::map_msgs INTERFACE
    map_msgs::map_msgs__rosidl_generator_cpp
    map_msgs::map_msgs__rosidl_typesupport_cpp)
endif()

if(NOT TARGET sensor_msgs::sensor_msgs)
  add_library(sensor_msgs::sensor_msgs INTERFACE IMPORTED)
  target_link_libraries(sensor_msgs::sensor_msgs INTERFACE
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
endif()

if(NOT TARGET tf2_msgs::tf2_msgs)
  add_library(tf2_msgs::tf2_msgs INTERFACE IMPORTED)
  target_link_libraries(tf2_msgs::tf2_msgs INTERFACE
    tf2_msgs::tf2_msgs__rosidl_generator_cpp
    tf2_msgs::tf2_msgs__rosidl_typesupport_cpp)
endif()

if(NOT TARGET nav2_util::nav2_util_core)
  message(STATUS "DEBUG: nav2_util_INCLUDE_DIRS = ${nav2_util_INCLUDE_DIRS}")
  message(STATUS "DEBUG: nav2_util_LIBRARIES = ${nav2_util_LIBRARIES}")
  add_library(nav2_util::nav2_util_core INTERFACE IMPORTED)
  target_link_libraries(nav2_util::nav2_util_core INTERFACE ${nav2_util_LIBRARIES})
  target_include_directories(nav2_util::nav2_util_core INTERFACE ${nav2_util_INCLUDE_DIRS})
endif()

if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
  if(TARGET yaml-cpp)
    target_link_libraries(yaml-cpp::yaml-cpp INTERFACE yaml-cpp)
  elseif(YAML_CPP_LIBRARIES)
    target_link_libraries(yaml-cpp::yaml-cpp INTERFACE ${YAML_CPP_LIBRARIES})
  else()
    target_link_libraries(yaml-cpp::yaml-cpp INTERFACE yaml-cpp)
  endif()
endif()



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
  SYSTEM
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_include_directories(${map_io_library_name}
  SYSTEM
  PRIVATE
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})
target_link_libraries(${map_io_library_name} PUBLIC
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  nav_msgs::nav_msgs
)
target_link_libraries(${map_io_library_name} PRIVATE
  ${GRAPHICSMAGICKCPP_LIBRARIES}
  tf2::tf2
  yaml-cpp::yaml-cpp
  Eigen3::Eigen
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
  nav_msgs::nav_msgs
  nav2_msgs::nav2_msgs
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_link_libraries(${library_name} PRIVATE
  lifecycle_msgs::lifecycle_msgs
  rclcpp_components::component
  yaml-cpp::yaml-cpp
)

add_library(${vo_library_name} SHARED
  src/vo_server/vector_object_shapes.cpp
  src/vo_server/vector_object_server.cpp)
target_include_directories(${vo_library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>")
target_link_libraries(${vo_library_name} PUBLIC
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  nav2_msgs::nav2_msgs
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  tf2_ros::tf2_ros
  ${UUID_LIBRARIES}
)
target_link_libraries(${library_name} PRIVATE
  lifecycle_msgs::lifecycle_msgs
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
  nav2_ros_common::nav2_ros_common
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
  nav_msgs::nav_msgs
  nav2_msgs::nav2_msgs
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
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
  nav_msgs::nav_msgs
  nav2_msgs::nav2_msgs
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
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
  nav_msgs::nav_msgs
  nav2_msgs::nav2_msgs
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

add_executable(${vo_server_executable}
  src/vo_server/main.cpp)
target_include_directories(${vo_server_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${vo_server_executable} PRIVATE
  ${vo_library_name}
  nav_msgs::nav_msgs
  nav2_msgs::nav2_msgs
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
ament_export_dependencies(nav_msgs nav2_msgs nav2_util rclcpp rclcpp_lifecycle nav2_ros_common Eigen3)
ament_export_targets(${library_name})

ament_package()
