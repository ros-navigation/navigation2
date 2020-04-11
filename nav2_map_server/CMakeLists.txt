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

nav2_package()

include_directories(include)

set(map_server_executable map_server)

set(map_saver_executable map_saver)

add_executable(${map_server_executable}
  src/main.cpp)

add_executable(${map_saver_executable}
  src/map_saver_cli.cpp)

set(library_name ${map_server_executable}_core)

add_library(${library_name} SHARED
  src/occ_grid_loader.cpp
  src/map_server.cpp
  src/map_saver.cpp
  src/map_mode.cpp)

target_compile_definitions(${library_name} PRIVATE "NAV2_MAP_SERVER_BUILDING_DLL")

set(map_server_dependencies
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  nav2_msgs
  yaml_cpp_vendor
  std_msgs
  tf2
  nav2_util)

set(map_saver_dependencies
  rclcpp
  nav_msgs
  tf2)

ament_target_dependencies(${map_server_executable}
  ${map_server_dependencies})

ament_target_dependencies(${map_saver_executable}
  ${map_saver_dependencies})

ament_target_dependencies(${library_name}
  ${map_server_dependencies})

target_link_libraries(${map_server_executable}
  ${library_name})

target_link_libraries(${map_saver_executable}
  ${library_name})

target_include_directories(${library_name} SYSTEM PRIVATE
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})

target_link_libraries(${library_name}
  ${GRAPHICSMAGICKCPP_LIBRARIES})

install(TARGETS ${map_server_executable} ${map_saver_executable}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include/)

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
)
ament_package()
