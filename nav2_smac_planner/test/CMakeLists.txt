# Test utils
ament_add_gtest(test_utils
  test_utils.cpp
)
target_link_libraries(test_utils
  ${library_name}
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
)

# Test costmap downsampler
ament_add_gtest(test_costmap_downsampler
  test_costmap_downsampler.cpp
)
target_link_libraries(test_costmap_downsampler
  ${library_name}
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  rclcpp::rclcpp
)

# Test Node2D
ament_add_gtest(test_node2d
  test_node2d.cpp
)
target_link_libraries(test_node2d
  ${library_name}
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test NodeHybrid
ament_add_gtest(test_nodehybrid
  test_nodehybrid.cpp
)
target_link_libraries(test_nodehybrid
  ${library_name}
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test NodeBasic
ament_add_gtest(test_nodebasic
  test_nodebasic.cpp
)
target_link_libraries(test_nodebasic
  ${library_name}
  rclcpp::rclcpp
)

# Test collision checker
ament_add_gtest(test_collision_checker
  test_collision_checker.cpp
)
target_link_libraries(test_collision_checker
  ${library_name}
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test A*
ament_add_gtest(test_a_star
  test_a_star.cpp
)
target_link_libraries(test_a_star
  ${library_name}
  ament_index_cpp::ament_index_cpp
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test SMAC Hybrid
ament_add_gtest(test_smac_hybrid
  test_smac_hybrid.cpp
)
target_link_libraries(test_smac_hybrid
  ${library_name}
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test SMAC 2D
ament_add_gtest(test_smac_2d
  test_smac_2d.cpp
)
target_link_libraries(test_smac_2d
  ${library_name}_2d
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test SMAC lattice
ament_add_gtest(test_smac_lattice
  test_smac_lattice.cpp
)
target_link_libraries(test_smac_lattice
  ${library_name}_lattice
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test SMAC Smoother
ament_add_gtest(test_smoother
  test_smoother.cpp
)
target_link_libraries(test_smoother
  ${library_name}_lattice
  ${library_name}
  ${library_name}_2d
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test Lattice node
ament_add_gtest(test_lattice_node test_nodelattice.cpp)
target_link_libraries(test_lattice_node
  ${library_name}
  ament_index_cpp::ament_index_cpp
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)


# Test Goal Manager
ament_add_gtest(test_goal_manager test_goal_manager.cpp)
target_link_libraries(test_goal_manager
  ${library_name}
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
