ament_add_gtest(vtest velocity_iterator_test.cpp)
target_link_libraries(vtest standard_traj_generator)

ament_add_gtest(twist_gen_test twist_gen.cpp)
target_link_libraries(twist_gen_test
  standard_traj_generator
  dwb_core::dwb_core
  ${dwb_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_2d_msgs_TARGETS}
  rclcpp::rclcpp
)

ament_add_gtest(kinematic_parameters_test kinematic_parameters_test.cpp)
target_link_libraries(kinematic_parameters_test
  standard_traj_generator
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
)

ament_add_gtest(speed_limit_test speed_limit_test.cpp)
target_link_libraries(speed_limit_test
  standard_traj_generator
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
)
