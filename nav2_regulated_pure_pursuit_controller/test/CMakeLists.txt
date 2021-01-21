# tests for regulated PP
ament_add_gtest(test_regulated_pp
  test_regulated_pp.cpp
)
ament_target_dependencies(test_regulated_pp
  ${dependencies}
)
target_link_libraries(test_regulated_pp
  ${library_name}
)
