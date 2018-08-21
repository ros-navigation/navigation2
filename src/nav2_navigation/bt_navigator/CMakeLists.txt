cmake_minimum_required(VERSION 3.5)
project(bt_navigator)

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

include_directories(
  include
  ../../nav2_tasks/include
  ../../nav2_robot/include
  ../../nav2_planning/include
  ../../nav2_control/include
  )

link_directories(${CMAKE_BINARY_DIR}/../libs)

add_executable(bt_navigator
  src/BTNavigator.cpp
  src/main.cpp
  )

target_link_libraries(bt_navigator
  nav2_robot
  )

ament_target_dependencies(bt_navigator 
  rclcpp 
  std_msgs
  nav2_tasks
  nav2_planning
  nav2_control
  nav2_robot
  )

install(TARGETS bt_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
