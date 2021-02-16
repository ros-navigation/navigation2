ament_add_gtest(test_sample_diff_drive_odom_motion_model
  plugins/sample_motion_models/test_diff_drive_odom_motion_model.cpp
)
ament_target_dependencies(test_sample_diff_drive_odom_motion_model
  ${dependencies}
)
target_link_libraries(test_sample_diff_drive_odom_motion_model
  ${library_name}
  sample_motion_model
)

ament_add_gtest(simple_test
  simple_test.cpp
)
ament_target_dependencies(simple_test
  ${dependencies}
)
target_link_libraries(simple_test
  ${library_name}
)

ament_add_gtest(test_angle_utils
  test_angle_utils.cpp
  ../src/angle_utils.cpp
)
