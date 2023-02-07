# Test utilities and basic types
ament_add_gtest(test_utils_and_types
  test_utils_and_types.cpp
)
ament_target_dependencies(test_utils_and_types
  ${dependencies}
)
target_link_libraries(test_utils_and_types
  ${library_name}
)

# Test edge scorer + plugins
ament_add_gtest(test_edge_scorers
  test_edge_scorers.cpp
)
ament_target_dependencies(test_edge_scorers
  ${dependencies}
)
target_link_libraries(test_edge_scorers
  ${library_name} edge_scorers
)

# Test path converter
ament_add_gtest(test_path_converter
  test_path_converter.cpp
)
ament_target_dependencies(test_path_converter
  ${dependencies}
)
target_link_libraries(test_path_converter
  ${library_name}
)

# Test node spatial tree
ament_add_gtest(test_spatial_tree
  test_spatial_tree.cpp
)
ament_target_dependencies(test_spatial_tree
  ${dependencies}
)
target_link_libraries(test_spatial_tree
  ${library_name}
)

# Test operation manager + plugins
ament_add_gtest(test_operations
  test_operations.cpp
)
ament_target_dependencies(test_operations
  ${dependencies}
)
target_link_libraries(test_operations
  ${library_name} route_operations
)

# Test collision monitor seperately due to relative complexity
ament_add_gtest(test_collision_operation
  test_collision_operation.cpp
)
ament_target_dependencies(test_collision_operation
  ${dependencies}
)
target_link_libraries(test_collision_operation
  ${library_name} route_operations
)

# Test route planner
ament_add_gtest(test_route_planner
  test_route_planner.cpp
)
ament_target_dependencies(test_route_planner
  ${dependencies}
)
target_link_libraries(test_route_planner
  ${library_name} edge_scorers
)
