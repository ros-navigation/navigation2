find_package(ament_cmake_google_benchmark QUIET)

if(ament_cmake_google_benchmark_FOUND)
  ament_add_google_benchmark(bt_path_blackboard_benchmark
    bt_path_blackboard_benchmark.cpp
  )
  target_link_libraries(bt_path_blackboard_benchmark
    behaviortree_cpp::behaviortree_cpp
    ${nav_msgs_TARGETS}
  )
else()
  message(STATUS "ament_cmake_google_benchmark not found, skipping BT path benchmark")
endif()
