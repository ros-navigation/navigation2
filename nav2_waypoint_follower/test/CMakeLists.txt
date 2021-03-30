# Test costmap downsampler
ament_add_gtest(test_task_executors
  test_task_executors.cpp
)
ament_target_dependencies(test_task_executors
  ${dependencies}
)
target_link_libraries(test_task_executors
  ${library_name} wait_at_waypoint photo_at_waypoint input_at_waypoint
)
