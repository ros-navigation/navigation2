cmake_minimum_required(VERSION 3.5)
project(control)

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
find_package(task REQUIRED)

include_directories(
  include
  ../task/include
  ../robot/include
  )

link_directories(${CMAKE_BINARY_DIR}/../libs)

add_executable(dwa_controller
  src/ControlTask.cpp
  src/DwaController.cpp
  src/main.cpp
  )

target_link_libraries(dwa_controller
  robot
  task
  )

ament_target_dependencies(dwa_controller 
  rclcpp 
  std_msgs
  task
  robot
  )

install(TARGETS dwa_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
