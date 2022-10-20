cmake_minimum_required(VERSION 3.5)
project(nav2_voxel_grid)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)

nav2_package()

include_directories(
  include)

add_library(voxel_grid SHARED
  src/voxel_grid.cpp
)

set(dependencies
  rclcpp
)

ament_target_dependencies(voxel_grid
  ${dependencies}
)

install(TARGETS voxel_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_dependencies(rclcpp)
ament_export_include_directories(include)
ament_export_libraries(voxel_grid)

ament_package()
