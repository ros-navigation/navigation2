find_package(benchmark REQUIRED)

set(BENCHMARK_NAMES
  optimizer_benchmark
  controller_benchmark
)

foreach(name IN LISTS BENCHMARK_NAMES)
  add_executable(${name}
    ${name}.cpp
  )
  ament_target_dependencies(${name}
    rclcpp
    geometry_msgs
    nav2_core
    nav2_costmap_2d
    nav_msgs
  )
  target_link_libraries(${name}
    benchmark
    mppi_controller
    mppi_critics
  )

  target_include_directories(${name} PRIVATE
    ${PROJECT_SOURCE_DIR}/test/utils
  )
endforeach()
