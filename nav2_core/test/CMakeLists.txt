# NavigatorMuxer test
nav2_add_gtest(test_navigator_muxer test_navigator_muxer.cpp)
target_link_libraries(test_navigator_muxer
  nav2_core
)

# BehaviorTreeNavigator test
nav2_add_gtest(test_behavior_tree_navigator test_behavior_tree_navigator.cpp)
target_link_libraries(test_behavior_tree_navigator
  nav2_core
  nav2_msgs::nav2_msgs
)

# Controller exceptions test
nav2_add_gtest(test_controller_exceptions test_controller_exceptions.cpp)
target_link_libraries(test_controller_exceptions
  nav2_core
)

# Planner exceptions test
nav2_add_gtest(test_planner_exceptions test_planner_exceptions.cpp)
target_link_libraries(test_planner_exceptions
  nav2_core
)
