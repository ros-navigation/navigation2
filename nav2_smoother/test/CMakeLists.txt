ament_add_gtest(test_smoother_server
  test_smoother_server.cpp
)

target_link_libraries(test_smoother_server
  ${library_name}
)

ament_target_dependencies(test_smoother_server
  ${dependencies}
)

ament_add_gtest(test_simple_smoother
  test_simple_smoother.cpp
)

target_link_libraries(test_simple_smoother
  simple_smoother
)

ament_target_dependencies(test_simple_smoother
  ${dependencies}
)


ament_add_gtest(test_savitzky_golay_smoother
  test_savitzky_golay_smoother.cpp
)

target_link_libraries(test_savitzky_golay_smoother
  savitzky_golay_smoother
)

ament_target_dependencies(test_savitzky_golay_smoother
  ${dependencies}
)
