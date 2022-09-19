cmake_minimum_required(VERSION 3.8)
project(nav2_fake_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)

set(DEPENDENCIES
  angles
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp
  geometry_msgs
  nav_msgs
  message_filters
)

include_directories(
  include
)

add_executable(fake_localization src/fake_localization.cpp)
ament_target_dependencies(fake_localization ${DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS fake_localization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
