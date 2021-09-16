# Test Node
ament_add_test(test_safety_node
  test_safety_node.cpp
)
ament_target_dependencies(test_safety_node
  ${dependencies}
)
target_link_libraries(test_safety_node
  ${library_name}
)