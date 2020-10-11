# Test costmap downsampler
ament_add_gtest(test_costmap_downsampler
  test_costmap_downsampler.cpp
)
ament_target_dependencies(test_costmap_downsampler
  ${dependencies}
)
target_link_libraries(test_costmap_downsampler
  ${library_name}
)

# Test Node2D
ament_add_gtest(test_node2d
  test_node2d.cpp
)
ament_target_dependencies(test_node2d
  ${dependencies}
)
target_link_libraries(test_node2d
  ${library_name}
)

# Test NodeSE2
ament_add_gtest(test_nodese2
  test_nodese2.cpp
)
ament_target_dependencies(test_nodese2
  ${dependencies}
)
target_link_libraries(test_nodese2
  ${library_name}
)

# Test NodeBasic
ament_add_gtest(test_nodebasic
  test_nodebasic.cpp
)
ament_target_dependencies(test_nodebasic
  ${dependencies}
)
target_link_libraries(test_nodebasic
  ${library_name}
)

# Test collision checker
ament_add_gtest(test_collision_checker
  test_collision_checker.cpp
)
ament_target_dependencies(test_collision_checker
  ${dependencies}
)
target_link_libraries(test_collision_checker
  ${library_name}
)

# Test A*
ament_add_gtest(test_a_star
  test_a_star.cpp
)
ament_target_dependencies(test_a_star
  ${dependencies}
)
target_link_libraries(test_a_star
  ${library_name}
)

# Test SMAC SE2
ament_add_gtest(test_smac_se2
  test_smac_se2.cpp
)
ament_target_dependencies(test_smac_se2
  ${dependencies}
)
target_link_libraries(test_smac_se2
  ${library_name}
)

# Test SMAC 2D
ament_add_gtest(test_smac_2d
  test_smac_2d.cpp
)
ament_target_dependencies(test_smac_2d
  ${dependencies}
)
target_link_libraries(test_smac_2d
  ${library_name}_2d
)

# Test smoother
ament_add_gtest(test_smoother
  test_smoother.cpp
)
ament_target_dependencies(test_smoother
  ${dependencies}
)
target_link_libraries(test_smoother
  ${library_name}_2d
)
