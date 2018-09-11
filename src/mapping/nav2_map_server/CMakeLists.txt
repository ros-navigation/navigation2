cmake_minimum_required(VERSION 3.5)
project(nav2_map_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rviz_yaml_cpp_vendor REQUIRED)

include_directories(
  include
  ${BULLET_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
)

add_library(map_reps
  src/map_reps/occupancy_grid.cpp
  src/map_reps/map_loader.cpp
  src/map_server_ros.cpp
)

ament_target_dependencies(map_reps rclcpp nav_msgs rviz_yaml_cpp_vendor)
target_link_libraries(map_reps
  ${BULLET_LIBRARIES}
  ${SDL_LIBRARY}
  ${SDL_IMAGE_LIBRARIES}
)

add_executable(map_server src/main.cpp)
ament_target_dependencies(map_server rclcpp nav_msgs)
target_link_libraries(map_server
  map_reps
)

install(TARGETS map_reps map_server DESTINATION lib/${PROJECT_NAME})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  # This forces cppcheck to consider all files in this project to be C++,
  # including the headers which end with .h, which cppcheck would normally
  # consider to be C instead.
  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_package()
