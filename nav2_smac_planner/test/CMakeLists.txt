# Test utils
ament_add_gtest(test_utils
  test_utils.cpp
)
ament_target_dependencies(test_utils
  ${dependencies}
)
target_link_libraries(test_utils
  ${library_name}
)

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

# Test NodeHybrid
ament_add_gtest(test_nodehybrid
  test_nodehybrid.cpp
)
ament_target_dependencies(test_nodehybrid
  ${dependencies}
)
target_link_libraries(test_nodehybrid
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

# Test SMAC Hybrid
ament_add_gtest(test_smac_hybrid
  test_smac_hybrid.cpp
)
ament_target_dependencies(test_smac_hybrid
  ${dependencies}
)
target_link_libraries(test_smac_hybrid
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

# Test SMAC lattice
ament_add_gtest(test_smac_lattice
  test_smac_lattice.cpp
)
ament_target_dependencies(test_smac_lattice
  ${dependencies}
)
target_link_libraries(test_smac_lattice
  ${library_name}_lattice
)

# Test SMAC Smoother
ament_add_gtest(test_smoother
  test_smoother.cpp
)
ament_target_dependencies(test_smoother
  ${dependencies}
)
target_link_libraries(test_smoother
  ${library_name}_lattice
  ${library_name}
  ${library_name}_2d
)

#Test Lattice node
ament_add_gtest(test_lattice_node test_nodelattice.cpp)

ament_target_dependencies(test_lattice_node ${dependencies})

target_link_libraries(test_lattice_node ${library_name})
