cmake_minimum_required(VERSION 3.5)
project(nav_2d_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(PROJECT_DEPENDENCIES
  geometry_msgs
  nav_2d_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

include_directories(
    include
)

add_library(conversions src/conversions.cpp)
ament_target_dependencies(conversions
  ${PROJECT_DEPENDENCIES}
)

add_library(path_ops src/path_ops.cpp)
ament_target_dependencies(path_ops
  ${PROJECT_DEPENDENCIES}
)

install(TARGETS conversions path_ops
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(conversions path_ops)

ament_package()
