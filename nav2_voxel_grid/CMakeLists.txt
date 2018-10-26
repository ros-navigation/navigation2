cmake_minimum_required(VERSION 3.5)
project(voxel_grid)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS})

add_library(voxel_grid src/voxel_grid.cpp)
target_link_libraries(voxel_grid ${rclcpp_LIBRARIES})

ament_export_dependencies(
  rclcpp
)
ament_export_include_directories(include)
ament_export_libraries(voxel_grid)

ament_package()

install(TARGETS voxel_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(voxel_grid_tests test/voxel_grid_tests.cpp)
  target_link_libraries(voxel_grid_tests
    voxel_grid
    ${rclcpp_LIBRARIES}
  )
endif()
