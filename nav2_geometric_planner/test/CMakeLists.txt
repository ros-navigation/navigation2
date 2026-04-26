ament_add_gtest(test_bezier_solver test_bezier_solver.cpp)
target_include_directories(test_bezier_solver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../math
  ${CMAKE_CURRENT_SOURCE_DIR}/../utils
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_bezier_solver geometric_planner ${PLANNER_LINK_LIBS})

ament_add_gtest(test_cubic_spline_solver test_cubic_spline_solver.cpp)
target_include_directories(test_cubic_spline_solver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../math
  ${CMAKE_CURRENT_SOURCE_DIR}/../utils
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_cubic_spline_solver geometric_planner ${PLANNER_LINK_LIBS})

ament_add_gtest(test_clothoid_solver test_clothoid_solver.cpp)
target_include_directories(test_clothoid_solver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../math
  ${CMAKE_CURRENT_SOURCE_DIR}/../utils
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_clothoid_solver geometric_planner ${PLANNER_LINK_LIBS})
