add_executable(test_updown
  test_updown.cpp
)
target_link_libraries(test_updown PRIVATE
  ${geometry_msgs_TARGETS}
  nav2_lifecycle_manager::nav2_lifecycle_manager_core
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
)
target_include_directories(test_updown
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)


install(TARGETS test_updown RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES test_updown_launch.py DESTINATION share/${PROJECT_NAME})
