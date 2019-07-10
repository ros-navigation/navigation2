cmake_minimum_required(VERSION 3.5)
project(base_local_planner)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_dynamic_params REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# nav2_package()

include_directories(
  include
)

set(dependencies
  angles
  nav2_costmap_2d
  nav2_dynamic_params
  geometry_msgs
  nav_msgs
  nav2_msgs
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# messages
rosidl_generate_interfaces(base_local_planner
    "msg/Position2DInt.msg"
    DEPENDENCIES std_msgs
)

add_library(base_local_planner_core
	src/footprint_helper.cpp
	src/goal_functions.cpp
	src/map_cell.cpp
	src/map_grid.cpp
	src/map_grid_visualizer.cpp
	src/map_grid_cost_function.cpp
	src/latched_stop_rotate_controller.cpp
	src/local_planner_util.cpp
	src/odometry_helper_ros.cpp
	src/obstacle_cost_function.cpp
	src/oscillation_cost_function.cpp
	src/prefer_forward_cost_function.cpp
	src/point_grid.cpp
	src/costmap_model.cpp
	src/simple_scored_sampling_planner.cpp
	src/simple_trajectory_generator.cpp
	src/trajectory.cpp
	src/twirling_cost_function.cpp
	src/voxel_grid_model.cpp
)

ament_target_dependencies(base_local_planner_core
  ${dependencies}
)

add_library(trajectory_planner_ros
	src/trajectory_planner.cpp
	src/trajectory_planner_ros.cpp)
target_link_libraries(trajectory_planner_ros
     base_local_planner_core)

add_executable(point_grid src/point_grid.cpp)
ament_target_dependencies(point_grid
  ${dependencies}
)

install(TARGETS
    base_local_planner_core
    trajectory_planner_ros
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
  ament_add_gtest(base_local_planner_utest
    test/gtest_main.cpp
    test/utest.cpp
    test/velocity_iterator_test.cpp
    test/footprint_helper_test.cpp
    test/trajectory_generator_test.cpp
    test/map_grid_test.cpp)
  target_link_libraries(base_local_planner_utest
      base_local_planner_core trajectory_planner_ros
      )

  ament_add_gtest(line_iterator
      test/line_iterator_test.cpp)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(base_local_planner_core)

ament_package()
