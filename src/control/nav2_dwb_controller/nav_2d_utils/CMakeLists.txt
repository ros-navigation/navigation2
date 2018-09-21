cmake_minimum_required(VERSION 3.5)
project(nav_2d_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # TODO(crdelsey): add -Werror once tf2/utils.h is fixed
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
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
  ${dependencies}
)

add_library(path_ops src/path_ops.cpp)
ament_target_dependencies(path_ops
  ${dependencies}
)

install(TARGETS conversions path_ops
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(conversions path_ops)
ament_export_dependencies(${dependencies})

ament_package()
