ament_add_gtest(test_bezier_planner test_bezier_planner.cpp)
target_include_directories(test_bezier_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../BezierPlanner/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../utils
)
target_link_libraries(test_bezier_planner
  bezier_planner
  ${PLANNER_LINK_LIBS}
)
