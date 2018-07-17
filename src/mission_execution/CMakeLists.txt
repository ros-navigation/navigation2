cmake_minimum_required(VERSION 3.5)
project(mission_execution)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  /usr/local/include/yasmine
  /usr/local/include/yasmine/include
  /usr/local/include/yasmine/include_impl
  /usr/local/include/yasmine/essentials/include
#  /usr/local/include/yasmine/hermes
  )

add_executable(mission_execution
  src/main.cpp
  src/MissionExecution.cpp
  src/MissionExecutionStateMachine.cpp
  src/NavigationCommand.cpp
  )

target_link_libraries(mission_execution
  yasmine
  )

ament_target_dependencies(mission_execution 
  rclcpp 
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
