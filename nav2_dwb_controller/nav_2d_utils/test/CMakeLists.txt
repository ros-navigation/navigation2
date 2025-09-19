ament_add_gtest(2d_utils_tests 2d_utils_test.cpp)
target_link_libraries(2d_utils_tests
  conversions
  nav2_ros_common::nav2_ros_common
)
target_include_directories(2d_utils_tests
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_add_gtest(path_ops_tests path_ops_test.cpp)
target_link_libraries(path_ops_tests path_ops)
