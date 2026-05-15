find_package(ament_cmake_google_benchmark REQUIRED)
ament_add_google_benchmark(observation_buffer_benchmark observation_buffer_benchmark.cpp)
target_link_libraries(observation_buffer_benchmark
  nav2_costmap_2d_core
  layers
)

ament_add_google_benchmark(inflation_layer_updatecosts_benchmark inflation_layer_updatecosts_benchmark.cpp)
target_link_libraries(inflation_layer_updatecosts_benchmark
  nav2_costmap_2d_core
  layers
)

# Enable OpenMP for benchmark if enabled
if(ENABLE_OPENMP)
  target_compile_options(inflation_layer_updatecosts_benchmark PRIVATE -fopenmp)
  target_link_libraries(inflation_layer_updatecosts_benchmark OpenMP::OpenMP_CXX)
endif()

add_executable(costmap_type_adapter_benchmark costmap_type_adapter_benchmark.cpp)
target_link_libraries(costmap_type_adapter_benchmark
  nav2_costmap_2d_core
  ${rclcpp_LIBRARIES}
  ${nav2_msgs_LIBRARIES}
)
target_include_directories(costmap_type_adapter_benchmark PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${nav2_msgs_INCLUDE_DIRS}
)
