ament_add_gtest(test_execution_timer test_execution_timer.cpp)

ament_add_gtest(test_node_utils test_node_utils.cpp)
target_link_libraries(test_node_utils ${library_name})
