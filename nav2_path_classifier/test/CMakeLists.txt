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

# Test constraint_classifier helper functions
ament_add_gtest(test_constraint_classifier_helpers
  test_constraint_classifier_helpers.cpp
)
ament_target_dependencies(test_constraint_classifier_helpers
  ${dependencies}
)
target_link_libraries(test_constraint_classifier_helpers
  pose_classifiers
  ${POLYCLIPPING_LIBRARIES}
)
