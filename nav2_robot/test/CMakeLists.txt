include_directories( ${PROJECT_SOURCE_DIR}/src )

ament_add_gtest(test_robot_class
  test_robot_class.cpp
)
ament_target_dependencies(test_robot_class
  ${dependencies}
)
target_link_libraries(test_robot_class nav2_robot)

