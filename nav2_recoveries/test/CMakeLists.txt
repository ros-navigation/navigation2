ament_add_gtest(test_recoveries
  test_recoveries.cpp
)

ament_target_dependencies(test_recoveries
  ${dependencies}
)
