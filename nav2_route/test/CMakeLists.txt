# Route planner benchmarking script
add_executable(performance_benchmarking performance_benchmarking.cpp)
target_link_libraries(performance_benchmarking
  ${library_name}
)
install(TARGETS
  performance_benchmarking
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Test utilities and basic types
ament_add_gtest(test_utils_and_types
  test_utils_and_types.cpp
)
target_link_libraries(test_utils_and_types
  ${library_name}
)

# Test edge scorer + plugins
ament_add_gtest(test_edge_scorers
  test_edge_scorers.cpp
)
target_link_libraries(test_edge_scorers
  ${library_name} edge_scorers
)

# Test path converter
ament_add_gtest(test_path_converter
  test_path_converter.cpp
)
target_link_libraries(test_path_converter
  ${library_name}
)

# Test node spatial tree
ament_add_gtest(test_spatial_tree
  test_spatial_tree.cpp
)
target_link_libraries(test_spatial_tree
  ${library_name}
)

# Test operation manager + plugins
ament_add_gtest(test_operations
  test_operations.cpp
)
target_link_libraries(test_operations
  ${library_name} route_operations
)

# Test graph loader
ament_add_gtest(test_graph_loader
    test_graph_loader.cpp
)
target_link_libraries(test_graph_loader
    ${library_name}
)

# Test graph saver
ament_add_gtest(test_graph_saver
    test_graph_saver.cpp
)
target_link_libraries(test_graph_saver
    ${library_name}
)

# Test geojson parser
ament_add_gtest(test_geojson_graph_file_loader
    test_geojson_graph_file_loader.cpp
)
target_link_libraries(test_geojson_graph_file_loader
  ${library_name} graph_file_loaders
)

# Test geojson saver
ament_add_gtest(test_geojson_graph_file_saver
    test_geojson_graph_file_saver.cpp
)
target_link_libraries(test_geojson_graph_file_saver
  ${library_name} graph_file_loaders graph_file_savers
)

# Test collision monitor separately due to relative complexity
ament_add_gtest(test_collision_operation
  test_collision_operation.cpp
)
target_link_libraries(test_collision_operation
  ${library_name} route_operations
)

# Test route planner
ament_add_gtest(test_route_planner
  test_route_planner.cpp
)
target_link_libraries(test_route_planner
  ${library_name} edge_scorers
)

# Test goal intent extractor
ament_add_gtest(test_goal_intent_extractor
  test_goal_intent_extractor.cpp
)
target_link_libraries(test_goal_intent_extractor
  ${library_name}
)

# Test route tracker
ament_add_gtest(test_route_tracker
  test_route_tracker.cpp
)
target_link_libraries(test_route_tracker
  ${library_name}
)

# Test route server
ament_add_gtest(test_route_server
  test_route_server.cpp
)
target_link_libraries(test_route_server
  ${library_name}
)

# Test goal intent search
ament_add_gtest(test_goal_intent_search
  test_goal_intent_search.cpp
)
target_link_libraries(test_goal_intent_search
  ${library_name}
)

# Test corner smoothing
ament_add_gtest(test_corner_smoothing
  test_corner_smoothing.cpp
)
target_link_libraries(test_corner_smoothing
  ${library_name}
)
