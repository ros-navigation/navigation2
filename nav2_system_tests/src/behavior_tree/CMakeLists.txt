find_package(Boost COMPONENTS system filesystem REQUIRED)

ament_add_gtest(test_behavior_tree_node
  test_behavior_tree_node.cpp
  server_handler.cpp
)

ament_target_dependencies(test_behavior_tree_node
  ${dependencies}
)

target_include_directories(test_behavior_tree_node PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(test_behavior_tree_node
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)
