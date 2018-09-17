include_directories( ${PROJECT_SOURCE_DIR}/src )

ament_add_gtest(test_robot_proxy test_robot_proxy.cpp)
ament_target_dependencies( test_robot_proxy
  ${dependencies}
)

ament_add_gtest(test_dwa_trajectory_generator test_dwa_trajectory_generator.cpp)
ament_target_dependencies( test_dwa_trajectory_generator
  ${dependencies}
)

ament_add_gtest(test_dwb_local_planner test_dwb_local_planner.cpp)
ament_target_dependencies( test_dwb_local_planner
  ${dependencies}
)
