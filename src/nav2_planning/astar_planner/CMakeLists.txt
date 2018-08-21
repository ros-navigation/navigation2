cmake_minimum_required(VERSION 3.5)
project(astar_planner)

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
  )

link_directories(${CMAKE_BINARY_DIR}/../libs)

add_executable(astar_planner
  src/AStarPlanner.cpp
  src/main.cpp
  )

target_link_libraries(astar_planner
  )

ament_target_dependencies(astar_planner 
  rclcpp 
  std_msgs
  nav2_tasks
  )

install(TARGETS astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
