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
    ${geometry_msgs_TARGETS}
    mppi_controller
    mppi_critics
    nav2_core::nav2_core
    nav2_costmap_2d::nav2_costmap_2d_core
    ${nav_msgs_TARGETS}
  )

  target_include_directories(${name} PRIVATE
    ${PROJECT_SOURCE_DIR}/test/utils
  )
endforeach()
