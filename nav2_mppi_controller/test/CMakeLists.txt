set(TEST_NAMES
  optimizer_smoke_test
  controller_state_transition_test
  models_test
  noise_generator_test
  parameter_handler_test
  motion_model_tests
  trajectory_visualizer_tests
  utils_test
  path_handler_test
  critic_manager_test
  optimizer_unit_tests
)

foreach(name IN LISTS TEST_NAMES)
  ament_add_gtest(${name}
    ${name}.cpp
  )
  target_link_libraries(${name}
    mppi_controller
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    nav2_costmap_2d::nav2_costmap_2d_core
    xtensor
    xtensor::optimize
    xtensor::use_xsimd
    nav2_core::nav2_core
  )

  if(${TEST_DEBUG_INFO})
    target_compile_definitions(${name} PUBLIC -DTEST_DEBUG_INFO)
  endif()

endforeach()

# This is a special case requiring linking against the critics library
ament_add_gtest(critics_tests critics_tests.cpp)
target_link_libraries(critics_tests mppi_controller mppi_critics rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle nav2_costmap_2d::nav2_costmap_2d_core xtensor xtensor::optimize xtensor::use_xsimd
)
if(${TEST_DEBUG_INFO})
  target_compile_definitions(critics_tests PUBLIC -DTEST_DEBUG_INFO)
endif()
