find_package(std_srvs REQUIRED)
find_package(test_msgs REQUIRED)

nav2_add_gtest(test_node_utils test_node_utils.cpp)
target_include_directories(test_node_utils PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_node_utils
  bondcpp::bondcpp
  geometry_msgs::geometry_msgs
  lifecycle_msgs::lifecycle_msgs
  nav2_msgs::nav2_msgs
  nav_msgs::nav_msgs
  rcl_interfaces::rcl_interfaces
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  pluginlib::pluginlib
)

nav2_add_gtest(test_service_client test_service_client.cpp)
target_include_directories(test_service_client PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_service_client
  bondcpp::bondcpp
  geometry_msgs::geometry_msgs
  lifecycle_msgs::lifecycle_msgs
  nav2_msgs::nav2_msgs
  nav_msgs::nav_msgs
  rcl_interfaces::rcl_interfaces
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  pluginlib::pluginlib
  std_srvs::std_srvs
)

nav2_add_gtest(test_service_server test_service_server.cpp)
target_include_directories(test_service_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_service_server
  bondcpp::bondcpp
  geometry_msgs::geometry_msgs
  lifecycle_msgs::lifecycle_msgs
  nav2_msgs::nav2_msgs
  nav_msgs::nav_msgs
  rcl_interfaces::rcl_interfaces
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  pluginlib::pluginlib
  std_srvs::std_srvs
)

nav2_add_gtest(test_actions test_actions.cpp)
target_include_directories(test_actions PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_actions
  bondcpp::bondcpp
  geometry_msgs::geometry_msgs
  lifecycle_msgs::lifecycle_msgs
  nav2_msgs::nav2_msgs
  nav_msgs::nav_msgs
  rcl_interfaces::rcl_interfaces
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  pluginlib::pluginlib
  test_msgs::test_msgs
)

nav2_add_gtest(test_lifecycle_node test_lifecycle_node.cpp)
target_include_directories(test_lifecycle_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_lifecycle_node
  bondcpp::bondcpp
  geometry_msgs::geometry_msgs
  lifecycle_msgs::lifecycle_msgs
  nav2_msgs::nav2_msgs
  nav_msgs::nav_msgs
  rcl_interfaces::rcl_interfaces
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  pluginlib::pluginlib
)

nav2_add_gtest(test_validation_messages test_validation_messages.cpp)
target_include_directories(test_validation_messages PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_validation_messages
  bondcpp::bondcpp
  geometry_msgs::geometry_msgs
  lifecycle_msgs::lifecycle_msgs
  nav2_msgs::nav2_msgs
  nav_msgs::nav_msgs
  map_msgs::map_msgs
  rcl_interfaces::rcl_interfaces
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  pluginlib::pluginlib
  builtin_interfaces::builtin_interfaces
  std_msgs::std_msgs
)
