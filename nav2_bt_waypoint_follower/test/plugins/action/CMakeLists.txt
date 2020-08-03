ament_add_gtest(test_action_get_next_goal_action
  test_get_next_goal_action.cpp
)
target_link_libraries(test_action_get_next_goal_action
  nav2_get_next_goal_action_bt_node
)
ament_target_dependencies(
  test_action_get_next_goal_action ${dependencies}
)
