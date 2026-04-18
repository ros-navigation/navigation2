ament_add_gtest(test_cubic_spline_planner test_cubic_spline_planner.cpp)
target_include_directories(test_cubic_spline_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../CubicSplinePlanner/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../utils
)
target_link_libraries(test_cubic_spline_planner
  cubic_spline_planner
  ${PLANNER_LINK_LIBS}
)
