include_directories(${PROJECT_SOURCE_DIR}/test)

# map_saver CLI
ament_add_gtest(test_map_saver_cli
  test_map_saver_cli.cpp
  ${PROJECT_SOURCE_DIR}/test/test_constants.cpp
)

ament_target_dependencies(test_map_saver_cli rclcpp nav_msgs)
target_link_libraries(test_map_saver_cli
$<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>
  ${dependencies}
)
