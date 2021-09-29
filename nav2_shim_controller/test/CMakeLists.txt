# tests for regulated PP
ament_add_gtest(test_shim_controller
test_shim_controller.cpp
)
ament_target_dependencies(test_shim_controller
  ${dependencies}
)
target_link_libraries(test_shim_controller
  ${library_name}
)
