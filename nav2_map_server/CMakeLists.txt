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

set(map_saver_cli_executable map_saver_cli)

set(map_saver_server_executable map_saver_server)

add_executable(${map_server_executable}
  src/map_server/main.cpp)

add_executable(${map_saver_cli_executable}
  src/map_saver/main_cli.cpp)

add_executable(${map_saver_server_executable}
  src/map_saver/main_server.cpp)

set(map_io_library_name map_io)

set(library_name ${map_server_executable}_core)

add_library(${map_io_library_name} SHARED
  src/map_mode.cpp
  src/map_io.cpp)

add_library(${library_name} SHARED
  src/map_server/map_server.cpp
  src/map_saver/map_saver.cpp)

set(map_io_dependencies
  yaml_cpp_vendor
  nav_msgs
  nav2_util
  tf2)

set(map_server_dependencies
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  nav2_msgs
  yaml_cpp_vendor
  std_msgs
  nav2_util)

set(map_saver_dependencies
  rclcpp
  nav_msgs
  nav2_msgs
  nav2_util)

ament_target_dependencies(${map_server_executable}
  ${map_server_dependencies})

ament_target_dependencies(${map_saver_cli_executable}
  ${map_saver_dependencies})

ament_target_dependencies(${map_saver_server_executable}
  ${map_saver_dependencies})

ament_target_dependencies(${library_name}
  ${map_server_dependencies})

ament_target_dependencies(${map_io_library_name}
  ${map_io_dependencies})

target_link_libraries(${library_name}
  ${map_io_library_name})

target_link_libraries(${map_server_executable}
  ${library_name})

if(WIN32)
  target_compile_definitions(${map_server_executable} PRIVATE
    YAML_CPP_DLL)
endif()

target_link_libraries(${map_saver_cli_executable}
  ${library_name})

target_link_libraries(${map_saver_server_executable}
  ${library_name})

target_include_directories(${map_io_library_name} SYSTEM PRIVATE
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})

target_link_libraries(${map_io_library_name}
  ${GRAPHICSMAGICKCPP_LIBRARIES})

if(WIN32)
  target_compile_definitions(${map_io_library_name} PRIVATE
    YAML_CPP_DLL)
endif()

install(TARGETS
    ${library_name} ${map_io_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
    ${map_server_executable} ${map_saver_cli_executable} ${map_saver_server_executable}
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
  ${map_io_library_name}
)
ament_package()
