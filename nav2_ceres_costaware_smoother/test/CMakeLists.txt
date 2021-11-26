ament_add_gtest(test_costaware_smoother
  test_costaware_smoother.cpp
)

target_link_libraries(test_costaware_smoother
  ${library_name}
)
ament_target_dependencies(test_costaware_smoother
  ${dependencies}
)
