find_package(benchmark REQUIRED)

add_executable(dstar_lite_benchmark dstar_lite_benchmark.cpp)
target_link_libraries(dstar_lite_benchmark
  ${PROJECT_NAME}
  benchmark::benchmark
  benchmark::benchmark_main
)
ament_target_dependencies(dstar_lite_benchmark
  nav2_core
  nav2_costmap_2d
  nav2_util
  rclcpp
  rclcpp_lifecycle
)
