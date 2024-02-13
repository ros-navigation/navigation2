ament_add_gtest(array_parser_test array_parser_test.cpp)
target_link_libraries(array_parser_test
  nav2_costmap_2d_core
)

ament_add_gtest(collision_footprint_test footprint_collision_checker_test.cpp)
target_link_libraries(collision_footprint_test
  ${PROJECT_NAME}::nav2_costmap_2d_core
)

ament_add_gtest(costmap_convesion_test costmap_conversion_test.cpp)
target_link_libraries(costmap_convesion_test
  ${PROJECT_NAME}::nav2_costmap_2d_core
)

ament_add_gtest(declare_parameter_test declare_parameter_test.cpp)
target_link_libraries(declare_parameter_test
  ${PROJECT_NAME}::nav2_costmap_2d_core
)

ament_add_gtest(costmap_filter_test costmap_filter_test.cpp)
target_link_libraries(costmap_filter_test
  ${PROJECT_NAME}::nav2_costmap_2d_core
)

ament_add_gtest(keepout_filter_test keepout_filter_test.cpp)
target_link_libraries(keepout_filter_test
  ${PROJECT_NAME}::nav2_costmap_2d_core
  ${PROJECT_NAME}::filters
)

ament_add_gtest(speed_filter_test speed_filter_test.cpp)
target_link_libraries(speed_filter_test
  ${PROJECT_NAME}::nav2_costmap_2d_core
  ${PROJECT_NAME}::filters
)

ament_add_gtest(binary_filter_test binary_filter_test.cpp)
target_link_libraries(binary_filter_test
  ${PROJECT_NAME}::nav2_costmap_2d_core
  ${PROJECT_NAME}::filters
)

ament_add_gtest(copy_window_test copy_window_test.cpp)
target_link_libraries(copy_window_test
  ${PROJECT_NAME}::nav2_costmap_2d_core
)

ament_add_gtest(costmap_filter_service_test costmap_filter_service_test.cpp)
target_link_libraries(costmap_filter_service_test
  ${PROJECT_NAME}::nav2_costmap_2d_core
)

ament_add_gtest(denoise_layer_test denoise_layer_test.cpp image_test.cpp image_processing_test.cpp)
target_link_libraries(denoise_layer_test
  ${PROJECT_NAME}::nav2_costmap_2d_core
  ${PROJECT_NAME}::layers
)
