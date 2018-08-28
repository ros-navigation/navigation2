cmake_minimum_required(VERSION 3.5)
project(nav2_bt_navigator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_tasks REQUIRED)
find_package(nav2_planning_msgs REQUIRED)

include_directories(
  include
  /usr/local/include/BTpp
)

add_executable(bt_navigator
  src/bt_navigator.cpp
  src/navigate_to_pose_behavior_tree.cpp
  src/main.cpp
)

target_link_libraries(bt_navigator
  /usr/local/BTpp/lib/libBTppLib.a
)

ament_target_dependencies(bt_navigator
  rclcpp
  std_msgs
  nav2_tasks
  nav2_planning_msgs
)

install(TARGETS bt_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
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

ament_package()
