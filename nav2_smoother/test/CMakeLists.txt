ament_add_gtest(test_smoother_server
  test_smoother_server.cpp
)

target_link_libraries(test_smoother_server
  ${library_name}
)
ament_target_dependencies(test_smoother_server
  ${dependencies}
)
