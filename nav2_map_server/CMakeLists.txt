cmake_minimum_required(VERSION 3.5)
project(nav2_map_server)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.10 REQUIRED common io)


nav2_package()

include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

set(map_server_executable map_server)

set(map_saver_cli_executable map_saver_cli)

set(map_saver_server_executable map_saver_server)

set(costmap_filter_info_server_executable costmap_filter_info_server)

add_executable(${map_server_executable}_2d
  src/map_server/main_2d.cpp)

add_executable(${map_server_executable}_3d
  src/map_server/main_3d.cpp)

add_executable(${map_saver_cli_executable}
  src/map_saver/main_cli.cpp)

add_executable(${map_saver_server_executable}_2d
  src/map_saver/main_server_2d.cpp)

add_executable(${map_saver_server_executable}_3d
  src/map_saver/main_server_3d.cpp)

add_executable(${costmap_filter_info_server_executable}
  src/costmap_filter_info/main.cpp)

set(map_io_library_name map_io)

set(library_name ${map_server_executable}_core)

add_library(${map_io_library_name}_2d SHARED
  src/map_2d/map_mode.cpp
  src/map_2d/map_io_2d.cpp)

add_library(${map_io_library_name}_3d SHARED
  src/map_3d/pcl_helper.cpp
  src/map_3d/map_io_3d.cpp)

add_library(${library_name} SHARED
  src/costmap_filter_info/costmap_filter_info_server.cpp
  include/nav2_map_server/map_server.hpp
  include/nav2_map_server/map_saver.hpp
  src/map_server/map_server_core.cpp
  src/map_saver/map_saver_core.cpp
  src/map_2d/map_server_2d.cpp
  src/map_2d/map_saver_2d.cpp
  src/map_3d/map_server_3d.cpp
  src/map_3d/map_saver_3d.cpp)

set(map_io_2d_dependencies
  rclcpp
  rclcpp_lifecycle
  yaml_cpp_vendor
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_util
  tf2)

set(map_io_3d_dependencies
  ${map_io_2d_dependencies}
  sensor_msgs
  geometry_msgs
  pcl_conversions)

set(map_server_dependencies
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  nav2_msgs
  geometry_msgs
  yaml_cpp_vendor
  std_msgs
  nav2_util)

set(map_saver_dependencies
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  nav2_msgs
  nav2_util
  geometry_msgs)

ament_target_dependencies(${map_server_executable}_2d
  ${map_server_dependencies})

ament_target_dependencies(${map_server_executable}_3d
  ${map_server_dependencies})

ament_target_dependencies(${map_saver_cli_executable}
  ${map_saver_dependencies})

ament_target_dependencies(${map_saver_server_executable}_2d
  ${map_saver_dependencies})

ament_target_dependencies(${map_saver_server_executable}_3d
  ${map_saver_dependencies})

ament_target_dependencies(${costmap_filter_info_server_executable}
  ${map_saver_dependencies})

ament_target_dependencies(${library_name}
  ${map_server_dependencies})

ament_target_dependencies(${map_io_library_name}_2d
  ${map_io_2d_dependencies})

ament_target_dependencies(${map_io_library_name}_3d
  ${map_io_3d_dependencies})

# linking pcl|eigen to nav2_io_3d_library
target_link_libraries(${map_io_library_name}_3d
  ${PCL_LIBRARIES}
  Eigen3::Eigen)

# interlinking libraries within package
target_link_libraries(${library_name}
  ${map_io_library_name}_2d
  ${map_io_library_name}_3d
  ${PCL_LIBRARIES})

target_link_libraries(${map_server_executable}_2d
  ${map_io_library_name}_2d
  ${library_name})

target_link_libraries(${map_server_executable}_3d
  ${map_io_library_name}_3d
  ${library_name})


if(WIN32)
  target_compile_definitions(${map_server_executable}_2d PRIVATE
    YAML_CPP_DLL)

  target_compile_definitions(${map_server_executable}_3d PRIVATE
    YAML_CPP_DLL)
endif()

target_link_libraries(${map_saver_cli_executable}
  ${library_name})

#target_link_libraries(${map_saver_cli_3d_executable}
#  ${library_name})

target_link_libraries(${map_saver_server_executable}_2d
  ${library_name})

target_link_libraries(${map_saver_server_executable}_3d
    ${library_name})

target_link_libraries(${costmap_filter_info_server_executable}
${library_name})

target_include_directories(${map_io_library_name}_2d SYSTEM PRIVATE
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})

target_link_libraries(${map_io_library_name}_2d
  ${GRAPHICSMAGICKCPP_LIBRARIES})

if(WIN32)
  target_compile_definitions(${map_io_library_name}_2d PRIVATE
    YAML_CPP_DLL)

  target_compile_definitions(${map_io_library_name}_3d PRIVATE
    YAML_CPP_DLL)
endif()

install(TARGETS
  ${library_name}
  ${map_io_library_name}_2d
  ${map_io_library_name}_3d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  ${map_server_executable}_2d
  ${map_server_executable}_3d
  ${map_saver_cli_executable}
  ${map_saver_server_executable}_2d
  ${map_saver_server_executable}_3d
  ${costmap_filter_info_server_executable}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

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
  ${map_io_library_name}_2d
  ${map_io_library_name}_3d
)
ament_package()
