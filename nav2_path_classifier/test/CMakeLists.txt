# Test the classification filter
ament_add_gtest(test_classification_filter
  test_classification_filter.cpp
)
ament_target_dependencies(test_classification_filter
  ${dependencies}
)
target_link_libraries(test_classification_filter
  ${library_name}
)

# Test the path classifier server
ament_add_gtest(test_path_classifier_server
  test_path_classifier_server.cpp
)
ament_target_dependencies(test_path_classifier_server
  ${dependencies}
)
target_link_libraries(test_path_classifier_server
  ${library_name}
)

# Test the visualization helpers
ament_add_gtest(test_utils
  test_utils.cpp
)
ament_target_dependencies(test_utils
  ${dependencies}
)

# Test the confined space classifier and its geometry helpers
ament_add_gtest(test_confined_space_classifier
  test_confined_space_classifier.cpp
)
ament_target_dependencies(test_confined_space_classifier
  ${dependencies}
)
target_link_libraries(test_confined_space_classifier
  pose_classifiers
  ${POLYCLIPPING_LIBRARIES}
)
