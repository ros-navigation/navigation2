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

#Test path validator
ament_add_gtest(test_path_validator
  test_path_validator.cpp
)
ament_target_dependencies(test_path_validator
  ${dependencies}
)
target_link_libraries(test_path_validator
  ${path_validator}
)