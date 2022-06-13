# Tests
ament_add_gtest(velocity_smoother_tests
  test_velocity_smoother.cpp
)
ament_target_dependencies(velocity_smoother_tests
  ${dependencies}
)
target_link_libraries(velocity_smoother_tests
  ${library_name}
)
