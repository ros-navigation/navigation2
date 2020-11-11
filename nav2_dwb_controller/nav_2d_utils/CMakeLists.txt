cmake_minimum_required(VERSION 3.5)
project(nav_2d_utils)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)

nav2_package()

set(dependencies
  geometry_msgs
  nav_2d_msgs
  nav_msgs
  std_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  nav2_msgs
  nav2_util
  nav_2d_msgs
)

include_directories(
    include
)

add_library(conversions SHARED
  src/conversions.cpp)

ament_target_dependencies(conversions
  ${dependencies}
)

add_library(path_ops SHARED
  src/path_ops.cpp)

ament_target_dependencies(path_ops
  ${dependencies}
)

add_library(tf_help SHARED
  src/tf_help.cpp
)

ament_target_dependencies(tf_help
  ${dependencies}
)

target_link_libraries(tf_help conversions)

install(TARGETS conversions path_ops tf_help
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(conversions path_ops tf_help)
ament_export_dependencies(${dependencies})

ament_package()
