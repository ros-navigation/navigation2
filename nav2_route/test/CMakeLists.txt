ament_add_gtest(test_bfs
    test_breadth_first_search.cpp
)
ament_target_dependencies(test_bfs
    ${dependencies}
)
target_link_libraries(test_bfs
    ${library_name} breadth_first_search
)
