ament_add_gtest(test_clothoid_planner test_clothoid_planner.cpp)
target_include_directories(test_clothoid_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../ClothoidPlanner/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../utils
)
target_link_libraries(test_clothoid_planner
  clothoid_planner
  ${PLANNER_LINK_LIBS}
)
