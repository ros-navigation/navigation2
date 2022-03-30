ament_add_gtest(test_costaware_smoother
  test_costaware_smoother.cpp
)

target_link_libraries(test_costaware_smoother
  ${library_name}
)
ament_target_dependencies(test_costaware_smoother
  ${dependencies}
)

ament_add_gtest(test_smoother_cost_function
  test_smoother_cost_function.cpp
)
target_link_libraries(test_smoother_cost_function
  ${library_name}
)
ament_target_dependencies(test_smoother_cost_function
  ${dependencies}
)