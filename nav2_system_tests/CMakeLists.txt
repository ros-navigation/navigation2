cmake_minimum_required(VERSION 3.5)
project(nav2_system_tests)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall) # -Wextra) # -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)

find_package(rclcpp REQUIRED)
find_package(nav2_tasks REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ament_lint_auto REQUIRED)

set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)

ament_add_gtest_executable(test_planner_node
  src/planning/test_planner_node.cpp
  src/planning/planner_tester.cpp
)

ament_target_dependencies(test_planner_node
  rclcpp
  nav2_tasks
  nav2_util
  nav2_msgs
  nav_msgs
  visualization_msgs
)

ament_add_test(test_planner_node
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/src/planning/test_planner_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_planner_node>
    TEST_MAP=${CMAKE_CURRENT_SOURCE_DIR}/maps/map.pgm
)

ament_lint_auto_find_test_dependencies()

endif()

ament_package()

