find_package(benchmark REQUIRED)

add_executable(dstar_lite_benchmark dstar_lite_benchmark.cpp)

if(TARGET nav2_core::nav2_core)
  target_link_libraries(dstar_lite_benchmark
    ${PROJECT_NAME}
    benchmark::benchmark
    benchmark::benchmark_main
    nav2_core::nav2_core
    nav2_costmap_2d::nav2_costmap_2d_core
    nav2_util::nav2_util_core
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )
else()
  target_link_libraries(dstar_lite_benchmark
    ${PROJECT_NAME}
    benchmark::benchmark
    benchmark::benchmark_main
    ${nav2_core_LIBRARIES}
    ${nav2_costmap_2d_LIBRARIES}
    ${nav2_util_LIBRARIES}
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )
endif()
