cmake_minimum_required(VERSION 3.5)
project(dwb_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(dwb_core REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
  angles
  dwb_core
  nav_2d_msgs
  nav_2d_utils
  pluginlib
  rclcpp)

include_directories(
  include
)

add_library(simple_goal_checker SHARED src/simple_goal_checker.cpp)
ament_target_dependencies(simple_goal_checker ${dependencies})

add_library(stopped_goal_checker SHARED src/stopped_goal_checker.cpp)
ament_target_dependencies(stopped_goal_checker simple_goal_checker ${dependencies})

add_library(standard_traj_generator SHARED
            src/standard_traj_generator.cpp
            src/limited_accel_generator.cpp
            src/kinematic_parameters.cpp
            src/xy_theta_iterator.cpp)
ament_target_dependencies(standard_traj_generator ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(vtest test/velocity_iterator_test.cpp)

  #find_package(rostest REQUIRED)
  add_executable(goal_checker test/goal_checker.cpp)
  target_link_libraries(goal_checker simple_goal_checker stopped_goal_checker ${GTEST_LIBRARIES})
  #add_rostest(test/goal_checker.launch DEPENDENCIES goal_checker)

  add_executable(twist_gen_test test/twist_gen.cpp)
  target_link_libraries(twist_gen_test standard_traj_generator ${GTEST_LIBRARIES})
  #add_rostest(test/twist_gen.launch DEPENDENCIES twist_gen_test)

endif()

install(TARGETS simple_goal_checker stopped_goal_checker standard_traj_generator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include/
)
install(FILES plugins.xml
        DESTINATION share
)

ament_export_include_directories(include)
ament_export_libraries(simple_goal_checker)
ament_export_libraries(stopped_goal_checker)
ament_export_libraries(standard_traj_generator)
pluginlib_export_plugin_description_file(dwb_core plugins.xml)

ament_package()
