ament_add_gtest(test_decorator_distance_controller
  plugins/decorator/test_distance_controller.cpp
)

target_link_libraries(test_decorator_distance_controller
  nav2_distance_controller_bt_node
)

ament_target_dependencies(test_decorator_distance_controller
  ${dependencies}
)
