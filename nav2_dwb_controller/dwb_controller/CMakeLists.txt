cmake_minimum_required(VERSION 3.5)
project(dwb_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_tasks REQUIRED)
find_package(nav2_robot REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(dwb_core REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

set(executable_name dwb_controller)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name}
  src/dwb_controller.cpp
)

set(dependencies
  rclcpp
  std_msgs
  nav2_tasks
  nav2_robot
  nav2_msgs
  dwb_core
)

ament_target_dependencies(${library_name}
  ${dependencies}
)
target_link_libraries(${library_name} ${Boost_LIBRARIES})

ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name} ${library_name} ${Boost_LIBRARIES})

install(TARGETS ${executable_name} ${library_name}
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
endif()

ament_export_include_directories(include)

ament_package()
