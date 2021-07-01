ament_add_gtest(test_recoveries
  test_recoveries.cpp
)

target_include_directories(test_recoveries PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

ament_target_dependencies(test_recoveries
  ${dependencies}
)
