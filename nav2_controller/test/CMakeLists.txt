# Test dynamic parameters
ament_add_gtest(test_controller_server
  test_controller_server.cpp
)
ament_target_dependencies(test_controller_server
  ${dependencies}
)
target_link_libraries(test_controller_server
  ${library_name}
)
