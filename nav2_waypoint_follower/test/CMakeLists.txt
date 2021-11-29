# Test test executors
ament_add_gtest(test_task_executors
  test_task_executors.cpp
)
ament_target_dependencies(test_task_executors
  ${dependencies}
)
target_link_libraries(test_task_executors
  ${library_name} wait_at_waypoint photo_at_waypoint input_at_waypoint
)

# Test dynamic parameters
ament_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
ament_target_dependencies(test_dynamic_parameters
  ${dependencies}
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
)
