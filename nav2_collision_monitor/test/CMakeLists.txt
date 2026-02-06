# Kinematics test
ament_add_gtest(kinematics_test kinematics_test.cpp)
ament_target_dependencies(kinematics_test
  ${dependencies}
)
target_link_libraries(kinematics_test
  ${monitor_library_name}
)

# Data sources test
ament_add_gtest(sources_test sources_test.cpp)
ament_target_dependencies(sources_test
  ${dependencies}
)
target_link_libraries(sources_test
  ${monitor_library_name}
  rclcpp::rclcpp
  nav2_util::nav2_util_core
  ${sensor_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  tf2_ros::tf2_ros
)

# Polygon shapes test
ament_add_gtest(polygons_test polygons_test.cpp)
ament_target_dependencies(polygons_test
  ${dependencies}
)
target_link_libraries(polygons_test
  ${monitor_library_name}
)

# Velocity Polygon test
ament_add_gtest(velocity_polygons_test velocity_polygons_test.cpp)
ament_target_dependencies(velocity_polygons_test
  ${dependencies}
)
target_link_libraries(velocity_polygons_test
  ${monitor_library_name}
)

# Collision Monitor node test
ament_add_gtest(collision_monitor_node_test collision_monitor_node_test.cpp)
ament_target_dependencies(collision_monitor_node_test
  ${dependencies}
)
target_link_libraries(collision_monitor_node_test
  ${monitor_library_name}
)
# Collision Detector node test
ament_add_gtest(collision_detector_node_test collision_detector_node_test.cpp)
ament_target_dependencies(collision_detector_node_test
  ${dependencies}
)
target_link_libraries(collision_detector_node_test
  ${detector_library_name}
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)


# Install bag + params so the launch file can find them via get_package_share_directory()
install(DIRECTORY bags/cm_moving_obstacle
  DESTINATION share/${PROJECT_NAME}/test/bags)
install(FILES collision_monitor_node_bag.yaml
  DESTINATION share/${PROJECT_NAME}/test)

find_package(GTest REQUIRED)

add_executable(collision_monitor_node_bag_gtest
  collision_monitor_node_bag.cpp
)
target_link_libraries(collision_monitor_node_bag_gtest
  GTest::gtest_main
  ${monitor_library_name}
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  nav2_util::nav2_util_core
  ${nav2_msgs_TARGETS}
)

add_launch_test(collision_monitor_node_bag.launch.py
  TIMEOUT 120
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  ENV TEST_EXECUTABLE=$<TARGET_FILE:collision_monitor_node_bag_gtest>
)
