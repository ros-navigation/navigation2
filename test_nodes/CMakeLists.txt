cmake_minimum_required(VERSION 3.5)
project(test_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_dynamic_params REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  tf2_ros
  tf2
  geometry_msgs
  tf2_geometry_msgs
  nav2_costmap_2d
  nav2_dynamic_params
)

add_executable(fake_tf2_transform src/fake_tf2_transform.cpp)
ament_target_dependencies(fake_tf2_transform
  ${dependencies}
)

add_executable(extra_param_node src/extra_param_node.cpp)
ament_target_dependencies(extra_param_node
  ${dependencies}
)

add_executable(param_tests src/param_tests.cpp)
ament_target_dependencies(param_tests
  ${dependencies}
)

install(TARGETS fake_tf2_transform param_tests extra_param_node
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

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # TODO(bpwilcox): Port/Add unit and component tests
  #add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_package()
