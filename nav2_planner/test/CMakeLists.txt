# Test dynamic parameters
ament_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
ament_target_dependencies(test_dynamic_parameters
  ${dependencies}
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
)

# Test path_splitter helper functions
ament_add_gtest(test_path_splitter_helpers
  test_path_splitter_helpers.cpp
)
ament_target_dependencies(test_path_splitter_helpers
  ${dependencies}
)
target_link_libraries(test_path_splitter_helpers
  ${library_name}
)
