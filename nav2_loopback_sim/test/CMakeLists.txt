# Tests
ament_add_gtest(loopback_simulator_tests
  test_loopback_simulator.cpp
)
ament_target_dependencies(loopback_simulator_tests
  ${dependencies}
)
target_link_libraries(loopback_simulator_tests
  ${library_name}
)