add_executable(test_updown
  test_updown.cpp
)
ament_target_dependencies(test_updown
  geometry_msgs
  nav2_lifecycle_manager
  rclcpp
)

install(TARGETS test_updown RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES test_updown_launch.py DESTINATION share/${PROJECT_NAME})
