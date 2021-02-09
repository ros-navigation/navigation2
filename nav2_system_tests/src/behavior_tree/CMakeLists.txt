ament_add_gtest(test_behavior_tree_node
  test_behavior_tree_node.cpp
  server_handler.cpp
)

ament_target_dependencies(test_behavior_tree_node
  ${dependencies}
)
