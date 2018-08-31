cmake_minimum_required(VERSION 3.5)
project(nav2_util)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_util_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_library(nav2_util STATIC
  src/costmap.cpp
)

ament_target_dependencies(nav2_util
  nav2_util_msgs
  tf2_ros
  tf2
)

install(TARGETS
  nav2_util
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include
)

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
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_util)

ament_package()
