# Test following server (unit)
ament_add_gtest(test_following_server_unit
  test_following_server_unit.cpp
)
target_link_libraries(test_following_server_unit
  ${geometry_msgs_TARGETS}
  ${library_name}
  nav2_util::nav2_util_core
  opennav_docking_core::opennav_docking_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test following server (smoke)
ament_add_pytest_test(test_following_server_topic test_following_server.py
  ENV
  FOLLOWING_MODE=topic
)

ament_add_pytest_test(test_following_server_frame test_following_server.py
  ENV
  FOLLOWING_MODE=frame
)

ament_add_pytest_test(test_following_server_skip_pose test_following_server.py
  ENV
  FOLLOWING_MODE=skip_pose
)

ament_add_pytest_test(test_following_server_search test_following_server.py
  ENV
  FOLLOWING_MODE=search
)
