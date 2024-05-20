add_executable(test_updown
  test_updown.cpp
)
target_link_libraries(test_updown
  ${geometry_msgs_TARGETS}
  nav2_lifecycle_manager::nav2_lifecycle_manager_core
  rclcpp::rclcpp
)

install(TARGETS test_updown RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES test_updown_launch.py DESTINATION share/${PROJECT_NAME})
