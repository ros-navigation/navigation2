find_package(std_srvs REQUIRED)
find_package(test_msgs REQUIRED)

ament_add_gtest(test_node_utils test_node_utils.cpp)
target_include_directories(test_node_utils PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_node_utils
  bondcpp::bondcpp
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
)

ament_add_gtest(test_service_client test_service_client.cpp)
target_include_directories(test_service_client PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_service_client
  bondcpp::bondcpp
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  ${std_srvs_TARGETS}
)

ament_add_gtest(test_service_server test_service_server.cpp)
target_include_directories(test_service_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_service_server
  bondcpp::bondcpp
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  ${std_srvs_TARGETS}
)

ament_add_gtest(test_actions test_actions.cpp)
target_include_directories(test_actions PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_actions
  bondcpp::bondcpp
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  ${test_msgs_TARGETS}
)

ament_add_gtest(test_lifecycle_node test_lifecycle_node.cpp)
target_include_directories(test_lifecycle_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_lifecycle_node
  bondcpp::bondcpp
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
)

ament_add_gtest(test_validation_messages test_validation_messages.cpp)
target_include_directories(test_validation_messages PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_validation_messages
  bondcpp::bondcpp
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  ${builtin_interfaces_TARGETS}
  ${std_msgs_TARGETS}
)
