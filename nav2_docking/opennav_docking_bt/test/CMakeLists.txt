# Cancel test
ament_add_gtest(test_dock_robot test_dock_robot.cpp)
target_link_libraries(test_dock_robot opennav_dock_action_bt_node)
ament_target_dependencies(test_dock_robot ${dependencies})

# Make command test
ament_add_gtest(test_undock_robot test_undock_robot.cpp)
target_link_libraries(test_undock_robot opennav_undock_action_bt_node)
ament_target_dependencies(test_undock_robot ${dependencies})
