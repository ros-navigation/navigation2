find_package(benchmark REQUIRED)

set(BENCHMARK_NAMES
  optimizer_benchmark
  controller_benchmark
)

foreach(name IN LISTS BENCHMARK_NAMES)
  add_executable(${name}
    ${name}.cpp
  )
  target_link_libraries(${name}
    benchmark
    geometry_msgs::geometry_msgs
    mppi_controller
    mppi_critics
    nav2_core::nav2_core
    nav2_costmap_2d::nav2_costmap_2d_core
    nav_msgs::nav_msgs
  )

  target_include_directories(${name} PRIVATE
    ${PROJECT_SOURCE_DIR}/test/utils
  )
endforeach()

# Closed-loop path tracking on simulated robots with execution delay, run manually
ament_add_gtest_executable(latency_benchmark latency_benchmark.cpp)
target_link_libraries(latency_benchmark
  mppi_controller
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
