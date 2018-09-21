cmake_minimum_required(VERSION 3.5)
project(nav2_planning_tests)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall) # -Wextra) # -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(nav2_tasks REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_libs_msgs REQUIRED)
find_package(nav2_planning_msgs REQUIRED)
find_package(nav2_world_model_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)

ament_add_gtest_executable(test_planner_node
  test_planner_node.cpp
  planner_tester.cpp
)

ament_target_dependencies(test_planner_node
  rclcpp
  nav2_tasks
  nav2_util
  nav2_libs_msgs
  nav2_planning_msgs
  nav2_world_model_msgs
  nav_msgs
  visualization_msgs
)

ament_add_test(test_planner_node
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_planner_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_planner_node>
    TEST_MAP=${CMAKE_CURRENT_SOURCE_DIR}/maps/map.pgm
)

install(TARGETS test_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
