# Tests
ament_add_gtest(controller_test
  controller_test.cpp
)
ament_target_dependencies(controller_test
  ${dependencies_pkgs}
)
target_link_libraries(controller_test
  mppic
)

ament_add_gtest(optimizer_test
  optimizer_test.cpp
)
ament_target_dependencies(optimizer_test
  ${dependencies_pkgs}
)
target_link_libraries(optimizer_test
  mppic
)
