# tests for regulated PP
ament_add_gtest(test_master_controller
test_master_controller.cpp
)
ament_target_dependencies(test_master_controller
  ${dependencies}
)
target_link_libraries(test_master_controller
  ${library_name}
)
