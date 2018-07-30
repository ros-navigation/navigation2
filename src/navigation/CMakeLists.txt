cmake_minimum_required(VERSION 3.5)
project(navigation)

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

include_directories(
  include
  ../task/include
  ../robot/include
  ../planning/include
  ../control/include
  )

link_directories(${CMAKE_BINARY_DIR}/../libs)

add_executable(simple_navigator
  src/SimpleNavigator.cpp
  src/main.cpp
  )

target_link_libraries(simple_navigator
  robot
  )

ament_target_dependencies(simple_navigator 
  rclcpp 
  std_msgs
  planning
  control
  robot
  )

install(TARGETS simple_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
