# Test following server (unit)
ament_add_gtest(test_following_server_unit
  test_following_server_unit.cpp
)
ament_target_dependencies(test_following_server_unit
  geometry_msgs
  nav2_util
  opennav_docking_core
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)
target_link_libraries(test_following_server_unit
  ${library_name}
)

# Test following server (smoke)
ament_add_pytest_test(test_following_server_topic test_following_server.py
  ENV
  FOLLOWING_MODE=topic
  TIMEOUT 120
)

ament_add_pytest_test(test_following_server_frame test_following_server.py
  ENV
  FOLLOWING_MODE=frame
  TIMEOUT 120
)

ament_add_pytest_test(test_following_server_skip_pose test_following_server.py
  ENV
  FOLLOWING_MODE=skip_pose
  TIMEOUT 120
)

ament_add_pytest_test(test_following_server_search test_following_server.py
  ENV
  FOLLOWING_MODE=search
  TIMEOUT 120
)
