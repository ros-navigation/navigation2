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
  ../libs/task/include
  )

link_directories(${CMAKE_BINARY_DIR}/../libs)

add_executable(navigation
  src/main.cpp
  src/Navigation.cpp
  )

ament_target_dependencies(navigation 
  rclcpp 
  std_msgs
  task
  )

target_link_libraries(navigation
  task
  yasmine
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
