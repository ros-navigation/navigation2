cmake_minimum_required(VERSION 3.5)
project(nav2_tasks)

# Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

#include_directories(
#  include
#)

install(DIRECTORY include/
  DESTINATION include/
)

rosidl_generate_interfaces(nav2_tasks
  "msg/MissionPlan.msg"
  "msg/PathEndPoints.msg"
  "msg/Path.msg"
  "msg/Costmap.msg"
  "msg/CostmapMetaData.msg"
  "srv/GetCostmap.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
