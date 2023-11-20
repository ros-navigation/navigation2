include_directories(${PROJECT_SOURCE_DIR}/test)

# map_io unit test
ament_add_gtest(test_map_io test_map_io.cpp ${PROJECT_SOURCE_DIR}/test/test_constants.cpp)

ament_target_dependencies(test_map_io rclcpp nav_msgs)

target_link_libraries(test_map_io
  ${map_io_library_name}
)

# costmap_filter_info_server unit test
ament_add_gtest(test_costmap_filter_info_server
  test_costmap_filter_info_server.cpp
)

ament_target_dependencies(test_costmap_filter_info_server rclcpp)

target_link_libraries(test_costmap_filter_info_server
  ${library_name}
)

# test_vector_object_shapes unit test
ament_add_gtest(test_vector_object_shapes
  test_vector_object_shapes.cpp
)

ament_target_dependencies(test_vector_object_shapes ${vo_server_dependencies})

target_link_libraries(test_vector_object_shapes
  ${vo_library_name}
)

# test_vector_object_server unit test
ament_add_gtest(test_vector_object_server
  test_vector_object_server.cpp
)

ament_target_dependencies(test_vector_object_server ${vo_server_dependencies})

target_link_libraries(test_vector_object_server
  ${vo_library_name}
)
