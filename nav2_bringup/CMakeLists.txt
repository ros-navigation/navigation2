cmake_minimum_required(VERSION 3.5)
project(nav2_bringup)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_smoother REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_recoveries REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

nav2_package()

set(executable_name composed_bringup)
add_executable(${executable_name}
  src/composed_bringup.cpp
)

set(dependencies
  nav2_map_server
  nav2_amcl
  nav2_controller
  nav2_smoother
  nav2_planner
  nav2_recoveries
  nav2_bt_navigator
  nav2_waypoint_follower
  nav2_lifecycle_manager
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
