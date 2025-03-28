ament_add_gtest(vtest velocity_iterator_test.cpp)
target_link_libraries(vtest standard_traj_generator)

ament_add_gtest(twist_gen_test twist_gen.cpp)
ament_target_dependencies(twist_gen_test
  dwb_core
  dwb_msgs
  geometry_msgs
  nav2_util
  nav_2d_msgs
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(twist_gen_test
  standard_traj_generator
)

ament_add_gtest(kinematic_parameters_test kinematic_parameters_test.cpp)
ament_target_dependencies(twist_gen_test
  rclcpp
  rcl_interfaces
)
target_link_libraries(kinematic_parameters_test
  standard_traj_generator
)

ament_add_gtest(speed_limit_test speed_limit_test.cpp)
ament_target_dependencies(speed_limit_test
  nav2_costmap_2d
  nav2_util
  rclcpp
)
target_link_libraries(speed_limit_test
  standard_traj_generator
)
