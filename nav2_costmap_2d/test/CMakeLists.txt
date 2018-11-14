ament_add_gtest(array_parser_test array_parser_test.cpp)
target_link_libraries(array_parser_test
  nav2_costmap_2d
)

ament_add_gtest(footprint_tests footprint_tests.cpp)
ament_target_dependencies(footprint_tests ${dependencies})
target_link_libraries(footprint_tests
  nav2_costmap_2d
)